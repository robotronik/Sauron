#ifndef _VIDCAP_H
#define _VIDCAP_H

#include <string>
#include <map>
#include <list>
#include <vector>
#include <cuda.h>
#include "lock.h"
#include <png.h>

/*
   The idea is as follows:
   Frames come as CUDA buffers, which are copied onto a device buffer in-sync.
   This buffer is continuously fed to a host buffer async.
   The host buffer is written into PNG pictures in one or more writer threads.

   The CUDA buffer should not overrun unless your PCI-E is slow, but if it does, you're screwed.
   The host buffer is expanded on-the-fly, so if you can't write PNGs out fast enough,
   you'll end up spilling on the swap.  If your disk is fast enough, it won't be a problem.
   If it isn't, you're screwed again.

   Be sure to have your CUDA initialized prior calling init() 
   (you can just start calling snap() at each frame without any manual initializations), 
   and have your CUDA context capable of unified addressing if you want to use 
   setSource with void*.
*/

class HostBuffer;
class DevBuffer;

class CUDAVidCap {
	public:
	CUDAVidCap(size_t devBufSize, int writers = 2);
	~CUDAVidCap();

	void setSource(void*); // You need unified addressing for this
	void floatSource(bool);
	void numComponents(int);
	void picPrefix(std::string);

	void dimensions(int, int);
	void setCrop(int, int, int, int);

	void snap();
	void wait(); // Wait till work is finished
		
	private:
	int d_startX, d_startY, d_endX, d_endY;
	int d_width, d_height;
	bool d_floatSource;
	int d_numComponents;
	std::string d_picPrefix;
	size_t d_bufSize;
	size_t d_frameSize, d_cropSize;
	bool d_running;
	int d_waitState;
	lockable d_waitLock;

	int d_readSlot; // The first slot of data not yet read
	int d_writeSlot; // Where will be written next
	int d_maxSlot; // The maximum number of slots
	int d_readPending; // Where to update readSlot when the transfer finishes
	int d_transferGranularity;
	int d_frameCounter;
	int *d_frameTable;
	void *d_source;
	void *d_devBuffer;
	void *d_hostPinned;

	void *d_hostMem;
	size_t d_hostMemSize;
	static const size_t g_reallocResolution = 128*1024*1024; // 128 MB
	std::list<int> d_freeSlots;
	std::list<std::pair<int,int> > d_usedSlots;
	lockable d_slotLock;
	std::vector<lockable> d_reallocLocks;
	int d_slotCounter;

	size_t d_hostSize;
	CUstream d_transferStream;
	CUevent d_transferEvent;

	lockable d_snapLock, d_storeLock;
	bool d_quit;

	void init();	
	void copyFrame(int,int);
	void writeFrame(void*, std::string);

	static void checkError(int, std::string desc = "");
	void spawnWorkers();
	static void *writerLaunch(void*);
	static void *copyLaunch(void*);
	void runWriter();
	void runCopy();
	int d_writers;
	std::vector<pthread_t> d_writerThread;
	pthread_t d_copyThread;
};

#endif
