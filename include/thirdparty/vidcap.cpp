#include "thirdparty/vidcap.h"
#include <map>

CUDAVidCap::CUDAVidCap(size_t bufSize, int writers) : 
	d_bufSize(bufSize),
	d_width(-1),
	d_height(-1),
	d_floatSource(false),
	d_numComponents(4),
	d_source(NULL),
	d_running(false),
	d_readSlot(-1),
	d_writeSlot(0),
	d_maxSlot(-1),
	d_frameSize(-1),
	d_hostSize(0),
	d_hostMem(NULL),
	d_startX(-1),
	d_startY(-1),
	d_endX(-1),
	d_endY(-1),
	d_writers(writers),
	d_waitState(0)
{
}

CUDAVidCap::~CUDAVidCap() {
	wait();

	// Deallocate host and dev mem
	checkError(cuStreamDestroy(d_transferStream), "Destroy CUDA stream");
	checkError(cuMemFree((CUdeviceptr)d_devBuffer), "Free dev buffer");
	checkError(cuMemFreeHost(d_hostPinned), "Free host pinned");
	free(d_frameTable);
	free(d_hostMem);
}

void CUDAVidCap::setSource(void *src) {
	d_source = src;
}

void CUDAVidCap::dimensions(int w, int h) {
	if (d_running && (w != d_width || h != d_height))
		throw std::string("Can't change dimensions while running");

	d_width = w;
	d_height = h;

	if (d_startX == -1 ||
			d_startY == -1 ||
			d_endX == -1 ||
			d_endY == -1) {
		d_startX = d_startY = 0;
		d_endX = d_width;
		d_endY = d_height;
	}
}

void CUDAVidCap::setCrop(int startX, int startY, int endX, int endY) {
	d_startX = startX;
	d_startY = startY;
	d_endX = endX;
	d_endY = endY;
}

void CUDAVidCap::floatSource(bool b) {
	if (d_running && (d_floatSource != b))
		throw std::string("Can't change source type while running");

	if (!b)
		throw std::string("Only float supported at the moment");

	d_floatSource = b;
}

void CUDAVidCap::numComponents(int c) {
	if (d_running && (d_numComponents != c))
		throw std::string("Can't change component count while running");

	if (c != 4)
		throw std::string("Only 4 components supported at the moment");

	d_numComponents = c;
}

void CUDAVidCap::picPrefix(std::string prefix) {
	d_picPrefix = prefix;
}

void CUDAVidCap::wait() {
	if (d_waitState)
		return;

	// Setting the wait state and waiting until all work is finished.
	// Wait state 1 == we signal snapping to seize, flushing buffers
	// Wait state 2 == copier signalled to finish
	// Wait state 3 == copier finished, workers signalled to finish
	// Wait state 4+n == n workers have finished

	// We're not going to call snap anymore
	// And we're waiting for any transfers to finish
	d_waitState = 1;
	if (d_readPending != d_readSlot) { // A transfer is on its way
		// Checking if it has finished
		if (cuEventSynchronize(d_transferEvent) == CUDA_SUCCESS) {

			// Setting the frame indices
			for (int i = d_readSlot; i < d_readPending; ++i)
				d_frameTable[i] = d_frameCounter++;

			d_readPending %= d_maxSlot;
			d_readSlot = d_readPending;


			// Signalling that we have new data
			d_snapLock.unLock();
		} else {
			throw std::string("cyEventSynchronize gave error");
		}
	}

	d_waitState = 2;
	d_snapLock.unLock(); // Signalling copier

	while (true) {
		d_waitLock.lock();
		if (d_waitState == d_writerThread.size() + 3) {
			printf("VidCap reached max wait state, done\n");
			return;
		}
	}
}

void CUDAVidCap::snap() {
	if (d_waitState)
		return; // Not doing anything if already exitting..  This should not occur though

	if (!d_running) {
		init();
		d_readSlot = 0;
		d_readPending = 0;
	}

	int nextSlot = (d_writeSlot+1)%d_maxSlot;

	// We drop frames instead of rewriting
	// A rewrite occurs when we are transfering and next slot is between d_readSlot and d_readPending-1 (inclusive)
	if (d_readPending != d_readSlot &&
			(nextSlot >= d_readSlot && nextSlot <= d_readPending-1)) {
		fprintf(stderr, "Would write onto slot %d, but we are transfering slots %d through %d.  Refusing to overwrite!\n",
			nextSlot, d_readSlot, d_readPending-1);
		exit(12);
	}

	// Now actually reading the buffer
	void *devStart = d_devBuffer + d_frameSize*d_writeSlot;
	checkError(cuMemcpy((CUdeviceptr)devStart, (CUdeviceptr)d_source, d_frameSize), "Copy within dev");

	d_writeSlot = nextSlot;

	// Deciding whether we start transferring data off from the GPU.  This can only happen is no transfer is already occurring.
	// (d_readPending == d_readSlot)
	// Firstly, if writeslot is behind us, we're wrapping around and just in case reading
	if (d_readPending == d_readSlot && (d_writeSlot < d_readSlot ||
		// Secondly, if writeslot is more than transferGranularity away
		d_writeSlot >= d_readSlot + d_transferGranularity)) {

		if (d_writeSlot < d_readSlot)
			d_readPending = d_maxSlot;
		else
			d_readPending = d_writeSlot;

		//printf("Transfering slots %d through %d\n", d_readSlot, d_readPending-1);

		int memOffset = d_readSlot*d_frameSize;
		checkError(cuMemcpyAsync((CUdeviceptr)(d_hostPinned + memOffset), 
					(CUdeviceptr)(d_devBuffer + memOffset), 
					(d_readPending-d_readSlot)*d_frameSize,
					d_transferStream), "memcpyAsync");
		checkError(cuEventRecord(d_transferEvent, d_transferStream), "Event record");
	}

	if (d_readPending != d_readSlot) { // A transfer is on its way
		// Checking if it has finished
		if (cuEventQuery(d_transferEvent) == CUDA_SUCCESS) {
			//printf("Transfer has finished\n");

			// Setting the frame indices
			for (int i = d_readSlot; i < d_readPending; ++i) {
				if (d_frameTable[i] != -1) {
					fprintf(stderr, "The previous frame is not read yet!  Frames lost!\n");
					exit(123);
				} else 
					d_frameTable[i] = d_frameCounter++;
			}

			d_readPending %= d_maxSlot;
			d_readSlot = d_readPending;

			// Signalling that we have new data
			d_snapLock.unLock();
		} /*else
			printf("Transfer NOT finished\n");*/
	}
}

void CUDAVidCap::init() {
	// We require unified addressing for this
	/*CUdevice *cuDev;
	checkError(cuCtxGetDevice(cuDev), "Get current device");
	int unified;
	checkError(cuDeviceGetAttribute(&unified, CU_DEVICE_ATTRIBUTE_UNIFIED_ADDRESSING, *cuDev), "Get attrib");
	if (!unified)
		throw std::string("Unified addressing needed for CUDAVidCap");*/

	d_running = true;

	d_frameSize = d_width*d_height*d_numComponents *
		(d_floatSource ? sizeof(float) : sizeof(unsigned char));

	d_cropSize = (d_endY - d_startY)*(d_endX - d_startX)*d_numComponents*sizeof(unsigned char);

	d_maxSlot = d_bufSize/d_frameSize;

	if (d_maxSlot < 2) 
		throw std::string("Not enough buffer to store a pair of frames");

	printf("We can buffer %d frames on the device\n", d_maxSlot);

	// Snapping to full frames
	if (d_maxSlot*d_frameSize != d_bufSize) {
		d_bufSize = d_maxSlot*d_frameSize;
		printf("Downgrading buffer size to %lu B\n", d_bufSize);
	}

	d_frameTable = (int*) malloc(sizeof(int)*d_maxSlot);
	for (int i = 0; i < d_maxSlot; ++i)
		d_frameTable[i] = -1;
	d_frameCounter = 0;
	
	CUdeviceptr devMem;
	checkError(cuMemAlloc(&devMem, d_bufSize), "Alloc dev mem");
	d_devBuffer = (void*) devMem;

	// Also allocating corresponding host buffer to perform async PCIE transfers
	checkError(cuMemAllocHost(&d_hostPinned, d_bufSize), "Alloc pinned host");
	
	d_transferGranularity = d_maxSlot/4;
	//d_transferGranularity = 1;
	if (!d_transferGranularity)
		d_transferGranularity = 1;

	// We create a new stream for the async transfers
	checkError(cuStreamCreate(&d_transferStream, 0), "Create CUDA stream");
	checkError(cuEventCreate(&d_transferEvent, CU_EVENT_DISABLE_TIMING), "Create event");

	spawnWorkers();
}

void CUDAVidCap::spawnWorkers() {
	for (int i = 0; i < d_writers; ++i)
		d_reallocLocks.push_back(lockable());

	if (pthread_create(&d_copyThread, NULL, copyLaunch, (void*)this))
		throw std::string("Could not spawn a thread");

	for (int i = 0; i < d_writers; ++i) {
		pthread_t tempThread;
		if (pthread_create(&tempThread, NULL, writerLaunch, (void*)this))
			throw std::string("Could not spawn a thread");
		d_writerThread.push_back(tempThread);
	}
}

void *CUDAVidCap::copyLaunch(void *cap) {
	try {
		((CUDAVidCap*)cap)->runCopy();
	} catch (std::string e) {
		fprintf(stderr, "VidCap copier %p:  error %s\n", pthread_self(), e.c_str());
		exit(4);
	}

	return NULL;
}

void *CUDAVidCap::writerLaunch(void *cap) {
	try {
		((CUDAVidCap*)cap)->runWriter();
	} catch (std::string e) {
		fprintf(stderr, "VidCap writer %p:  error %s\n", pthread_self(), e.c_str());
		exit(5);
	}
	return NULL;
}

void CUDAVidCap::runWriter() {
	d_storeLock.lock();

	lockable *myReallocLock = NULL;
	int writerId;

	// Such a hack :-I
	while (!myReallocLock) {
		for (int i = 0; i < d_writerThread.size(); ++i)
			if (d_writerThread.at(i) == pthread_self()) {
				printf("Writer number %d/%d spawned\n", i+1, d_writerThread.size());
				myReallocLock = &d_reallocLocks.at(i);
				writerId = i;
			}
		sleep(1);
	}

	while (true) {
		d_storeLock.lockTimeout(1000);

		// Flushing the entire slot buffer
		while (!d_usedSlots.empty()) {
			d_slotLock.lock();
			std::pair<int, int> slot = d_usedSlots.front();
			d_usedSlots.pop_front();
			d_slotLock.unLock();

			char fname[1024];
			sprintf(fname, "%s%06d.png", d_picPrefix.c_str(), slot.first);

			myReallocLock->lock();
			writeFrame((void*)(d_hostMem + slot.second*d_cropSize), std::string(fname));
			myReallocLock->unLock();

			// Every 10 frames we print progress
			if (!(slot.first%10))
				printf("VidCap:  frames in %d, frames out %d (processed %.1f%%)\n",
						d_frameCounter, slot.first, (float)slot.first/(float)d_frameCounter*100.0f);

			d_slotLock.lock();
			d_freeSlots.push_back(slot.second);
			d_slotLock.unLock();
		}

		// No more slots, and if wait state is on for us workers, we quit
		if (d_waitState > 2) {
			// Just temporarily using some lock (not used for signalling) here
			d_slotLock.lock();
			d_waitState++;
			d_slotLock.unLock();

			d_waitLock.unLock(); // Signalling main thread

			return;
		}
	}
}

void CUDAVidCap::writeFrame(void *data, std::string fname) {
	png_structp pngPtr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	png_infop infoPtr = png_create_info_struct(pngPtr);
	setjmp(png_jmpbuf(pngPtr));

	if (!pngPtr || !infoPtr)
		throw std::string("Failed to init libpng");

	int width = d_endX - d_startX;
	int height = d_endY - d_startY;

	png_set_IHDR(pngPtr, infoPtr, width, height,
			8, PNG_COLOR_TYPE_RGBA, PNG_INTERLACE_NONE,
			PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

	FILE *fp = fopen(fname.c_str(), "w");
	if (!fp)
		throw std::string("Couldn't open file " + fname + " for writing");
	
	png_init_io(pngPtr, fp);

	if (setjmp(png_jmpbuf(pngPtr)))
		throw std::string("Error while writing png (1)");

	png_write_info(pngPtr, infoPtr);

	if (setjmp(png_jmpbuf(pngPtr)))
		throw std::string("Error while writing png (2)");

	png_bytep *rowPtrs;
	rowPtrs = (png_bytep*) malloc(sizeof(png_bytep)*height);

	for (int i = 0; i < height; ++i)
		rowPtrs[i] = ((unsigned char*)data) + d_numComponents*width*i;

	png_write_image(pngPtr, rowPtrs);
	
	if (setjmp(png_jmpbuf(pngPtr)))
		throw std::string("Error while writing png (3)");

	png_write_end(pngPtr, NULL);

	free(rowPtrs);
	fclose(fp);
}

void CUDAVidCap::runCopy() {
	d_snapLock.lock();

	// Waiting for the first snap
	if (!d_writeSlot)
		d_snapLock.lock();

	int readFrameCount = 0;
	int missCounter = 0;
	
	while (true) {
		bool foundAny = false;
		bool found;
		do {
			found = false;
			for (int i = 0; i < d_maxSlot; ++i)
				if (d_frameTable[i] == readFrameCount) {
					found = true;
					d_frameTable[i] = -1;
					foundAny = true;
					missCounter = 0;
					copyFrame(readFrameCount, i);
					readFrameCount++;
				}
		} while (found);

		if (!foundAny)
			missCounter++;

		if (missCounter > 3) {
			fprintf(stderr, "WARNING!!!  Did not find frame %d at all!\n", readFrameCount);
			//readFrameCount++;
			exit(9);
		}

		// If we read all written slots and waitState is >1 (no more pending transfers), we go to the next wait state and signal
		if (d_waitState > 1) {
			if (missCounter) {
				d_waitState++;
				d_waitLock.unLock();
				return;
			}
		} else {
			// If wait state was not set, we're running normally, and waiting for a signal
			d_snapLock.lock();
		}
	}
}

void CUDAVidCap::copyFrame(int frameId, int framePos) {
	if (!d_hostMem) {
		d_hostMem = malloc(g_reallocResolution);
		d_hostMemSize = g_reallocResolution;

		for (d_slotCounter = 0; d_slotCounter < d_hostMemSize/d_cropSize; ++d_slotCounter) {
			d_slotLock.lock();
			d_freeSlots.push_back(d_slotCounter);
			d_slotLock.unLock();
		}
	}

	// Allocating more memory if we do not have any free slots
	if (d_freeSlots.empty()) {
		for (int i = 0; i < d_reallocLocks.size(); ++i)
			d_reallocLocks.at(i).lock();

		d_hostMemSize += g_reallocResolution;
		d_hostMem = realloc((void*)d_hostMem, d_hostMemSize);
		
		for (int i = 0; i < d_reallocLocks.size(); ++i)
			d_reallocLocks.at(i).unLock();

		for (; d_slotCounter < d_hostMemSize/d_cropSize; ++d_slotCounter) {
			d_slotLock.lock();
			d_freeSlots.push_back(d_slotCounter);
			d_slotLock.unLock();
		}

		printf("Upgraded slot pool to %d frames\n", d_slotCounter);
	}

	// Picking the first free slot
	d_slotLock.lock();
	int useSlot = d_freeSlots.front();
	d_freeSlots.pop_front();
	d_slotLock.unLock();

	// Copying the crop region
	int cropWidth = d_endX - d_startX;
	unsigned char *writePos = ((unsigned char*)d_hostMem) + (long long int)useSlot * (long long int)(cropWidth*(d_endY - d_startY)*d_numComponents);

	for (int y = d_startY; y < d_endY; ++y) {
		float *readPos = ((float*)d_hostPinned) + (framePos*d_width*d_height*d_numComponents +
			y*d_width*d_numComponents + d_startX*d_numComponents);
		for (int x = d_startX; x < d_endX; ++x)
			for (int c = 0; c < d_numComponents; ++c) {
				*writePos++ = (*readPos++)*255.0f + 0.499f;
			}
	}

	d_slotLock.lock();
	d_usedSlots.push_back(std::pair<int, int>(frameId, useSlot));
	d_slotLock.unLock();

	// Signalling the PNG writer
	d_storeLock.unLock();
}

void CUDAVidCap::checkError(int rCode, std::string desc) {
	static std::map<int, std::string> g_errorStrings;
	if (!g_errorStrings.size()) {
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_INVALID_VALUE, "CUDA_ERROR_INVALID_VALUE"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_OUT_OF_MEMORY, "CUDA_ERROR_OUT_OF_MEMORY"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_NOT_INITIALIZED, "CUDA_ERROR_NOT_INITIALIZED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_DEINITIALIZED, "CUDA_ERROR_DEINITIALIZED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_NO_DEVICE, "CUDA_ERROR_NO_DEVICE"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_INVALID_DEVICE, "CUDA_ERROR_INVALID_DEVICE"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_INVALID_IMAGE, "CUDA_ERROR_INVALID_IMAGE"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_INVALID_CONTEXT, "CUDA_ERROR_INVALID_CONTEXT"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_MAP_FAILED, "CUDA_ERROR_MAP_FAILED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_UNMAP_FAILED, "CUDA_ERROR_UNMAP_FAILED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_ARRAY_IS_MAPPED, "CUDA_ERROR_ARRAY_IS_MAPPED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_ALREADY_MAPPED, "CUDA_ERROR_ALREADY_MAPPED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_NO_BINARY_FOR_GPU, "CUDA_ERROR_NO_BINARY_FOR_GPU"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_ALREADY_ACQUIRED, "CUDA_ERROR_ALREADY_ACQUIRED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_NOT_MAPPED, "CUDA_ERROR_NOT_MAPPED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_NOT_MAPPED_AS_ARRAY, "CUDA_ERROR_NOT_MAPPED_AS_ARRAY"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_NOT_MAPPED_AS_POINTER, "CUDA_ERROR_NOT_MAPPED_AS_POINTER"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_UNSUPPORTED_LIMIT, "CUDA_ERROR_UNSUPPORTED_LIMIT"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_CONTEXT_ALREADY_IN_USE, "CUDA_ERROR_CONTEXT_ALREADY_IN_USE"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_INVALID_SOURCE, "CUDA_ERROR_INVALID_SOURCE"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_FILE_NOT_FOUND, "CUDA_ERROR_FILE_NOT_FOUND"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_SHARED_OBJECT_SYMBOL_NOT_FOUND, "CUDA_ERROR_SHARED_OBJECT_SYMBOL_NOT_FOUND"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_SHARED_OBJECT_INIT_FAILED, "CUDA_ERROR_SHARED_OBJECT_INIT_FAILED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_OPERATING_SYSTEM, "CUDA_ERROR_OPERATING_SYSTEM"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_INVALID_HANDLE, "CUDA_ERROR_INVALID_HANDLE"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_NOT_FOUND, "CUDA_ERROR_NOT_FOUND"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_NOT_READY, "CUDA_ERROR_NOT_READY"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_LAUNCH_FAILED, "CUDA_ERROR_LAUNCH_FAILED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_LAUNCH_OUT_OF_RESOURCES, "CUDA_ERROR_LAUNCH_OUT_OF_RESOURCES"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_LAUNCH_TIMEOUT, "CUDA_ERROR_LAUNCH_TIMEOUT"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_LAUNCH_INCOMPATIBLE_TEXTURING, "CUDA_ERROR_LAUNCH_INCOMPATIBLE_TEXTURING"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_PRIMARY_CONTEXT_ACTIVE, "CUDA_ERROR_PRIMARY_CONTEXT_ACTIVE"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_CONTEXT_IS_DESTROYED, "CUDA_ERROR_CONTEXT_IS_DESTROYED"));
		g_errorStrings.insert(std::pair<int, std::string>(CUDA_ERROR_UNKNOWN, "CUDA_ERROR_UNKNOWN"));
	}

	if (rCode != CUDA_SUCCESS)
		throw ((desc == "") ? 
				std::string("Error: ") : 
				(std::string("Error in \"") + desc + std::string("\": "))) + 
			g_errorStrings[rCode];
}
