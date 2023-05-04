#include "Cameras/VideoCaptureCamera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <stdlib.h>
#include <opencv2/imgproc.hpp>


#include <opencv2/calib3d.hpp>

#include "thirdparty/list-devices.hpp"
#include "thirdparty/serialib.h"

#include "data/Calibfile.hpp"
#include "data/FrameCounter.hpp"
#include "data/CameraView.hpp"

#include "TrackedObjects/TrackedObject.hpp" //CameraView
#include "ObjectTracker.hpp"
#include "GlobalConf.hpp"
#include "data/FVector2D.hpp"

using namespace cv;
using namespace std;

bool VideoCaptureCamera::StartFeed()
{
	auto globalconf = GetCaptureConfig();
	if (connected)
	{
		return false;
	}
	FrameBuffer.resize(Settings.BufferSize);
	for (int i = 0; i < Settings.BufferSize; i++)
	{
		FrameBuffer[i] = BufferedFrame();
	}

	string pathtodevice = Settings.DeviceInfo.device_paths[0];
	for (int i = 1; i < Settings.DeviceInfo.device_paths.size(); i++)
	{
		int curridx;
		int currread = sscanf(pathtodevice.c_str(), "/dev/video%d", &curridx);
		int newidx;
		int newread = sscanf(Settings.DeviceInfo.device_paths[i].c_str(), "/dev/video%d", &curridx);
		if (currread == 1 && newread == 1)
		{
			if (newidx < curridx)
			{
				pathtodevice = Settings.DeviceInfo.device_paths[i];
			}
		}
	}
	
	
	if (Settings.StartType == CameraStartType::CUDA)
	{
		Settings.StartPath = pathtodevice;
		Settings.ApiID = CAP_ANY;

		//d_reader = cudacodec::createVideoReader(Settings.DeviceInfo.device_paths[0], {
		//	CAP_PROP_FRAME_WIDTH, Settings.Resolution.width, 
		//	CAP_PROP_FRAME_HEIGHT, Settings.Resolution.height, 
		//	CAP_PROP_FPS, Settings.Framerate/Settings.FramerateDivider,
		//	CAP_PROP_BUFFERSIZE, 1});

	}
	else
	{
		ostringstream sizestream;
		sizestream << "width=(int)" << Settings.Resolution.width
				<< ", height=(int)" << Settings.Resolution.height;
		ostringstream sizestreamnocrop;
		sizestreamnocrop << "width=(int)" << Settings.Resolution.width + globalconf.CropRegion.x + globalconf.CropRegion.width
				<< ", height=(int)" << Settings.Resolution.height + globalconf.CropRegion.y + globalconf.CropRegion.height;
		switch (Settings.StartType)
		{
		case CameraStartType::GSTREAMER_NVARGUS:
			{
				//Settings.StartPath = "nvarguscamerasrc ! nvvidconv ! videoconvert ! appsink drop=1";
				ostringstream capnamestream;
				int sensorid = Settings.DeviceInfo.bus_info.back() == '4' ? 1 : 0;
				
				capnamestream << "nvarguscamerasrc sensor-id=" << sensorid << " ! video/x-raw(memory:NVMM), " 
				<< sizestreamnocrop.str() << ", framerate=(fraction)"
				<< (int)Settings.Framerate << "/" << (int)Settings.FramerateDivider 
				<< " ! nvvidconv flip-method=2 left=" << globalconf.CropRegion.x << " top=" << globalconf.CropRegion.y 
				<< " right=" << Settings.Resolution.width + globalconf.CropRegion.x 
				<< " bottom=" << Settings.Resolution.height + globalconf.CropRegion.y
				<< " ! video/x-raw, format=(string)BGRx, " << sizestream.str() << " ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1";
				Settings.StartPath = capnamestream.str();
				Settings.ApiID = CAP_GSTREAMER;
			}
			break;
		case CameraStartType::GSTREAMER_CPU:
		case CameraStartType::GSTREAMER_NVDEC:
		case CameraStartType::GSTREAMER_JETSON:
			{
				ostringstream capnamestream;
				capnamestream << "v4l2src device=" << pathtodevice << " io-mode=4 ! image/jpeg, width=" 
				<< Settings.Resolution.width << ", height=" << Settings.Resolution.height << ", framerate="
				<< (int)Settings.Framerate << "/" << (int)Settings.FramerateDivider << ", num-buffers=1 ! ";
				if (Settings.StartType == CameraStartType::GSTREAMER_CPU)
				{
					capnamestream << "jpegdec ! videoconvert";
				}
				else if (Settings.StartType == CameraStartType::GSTREAMER_JETSON)
				{
					capnamestream << "nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert";
				}
				else if(Settings.StartType == CameraStartType::CUDA)
				{
					capnamestream << "nvdec ! glcolorconvert ! gldownload";
				}
				capnamestream << " ! video/x-raw, format=BGR ! appsink drop=1";
				Settings.StartPath = capnamestream.str();
				Settings.ApiID = CAP_GSTREAMER;
			}
			break;
		default:
			cerr << "WARNING : Unrecognised Camera Start Type in VideoCaptureCamera, defaulting to auto API" << endl;
		case CameraStartType::ANY:
			Settings.StartPath = pathtodevice;
			Settings.ApiID = CAP_ANY;
			break;
		}
		char commandbuffer[1024];
		snprintf(commandbuffer, sizeof(commandbuffer), "v4l2-ctl -d %s -c exposure_auto=%d,exposure_absolute=%d", pathtodevice.c_str(), 1, 32);
		cout << "Aperture system command : " << commandbuffer << endl;
		system(commandbuffer);
		feed = new VideoCapture();
		cout << "Opening device at \"" << Settings.StartPath << "\" with API id " << Settings.ApiID << endl;
		feed->open(Settings.StartPath, Settings.ApiID);
		if (Settings.StartType == CameraStartType::ANY)
		{
			feed->set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
			//feed->set(CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
			feed->set(CAP_PROP_FRAME_WIDTH, Settings.Resolution.width);
			feed->set(CAP_PROP_FRAME_HEIGHT, Settings.Resolution.height);
			feed->set(CAP_PROP_FPS, Settings.Framerate/Settings.FramerateDivider);
			feed->set(CAP_PROP_AUTO_EXPOSURE, 1) ;
			//feed->set(CAP_PROP_EXPOSURE, 32) ;
			//feed->set(CAP_PROP_BUFFERSIZE, 1);
		}
		//cout << "Success opening ? " << feed->isOpened() << endl;
		/*if (!feed->isOpened())
		{
			exit(EXIT_FAILURE);
		}*/
		
	}
	
	connected = true;
	return true;
}

bool VideoCaptureCamera::Grab(int BufferIndex)
{
	if (!connected)
	{
		return false;
	}
	bool grabsuccess = false;
	grabsuccess = feed->grab();
	if (grabsuccess)
	{
		FrameBuffer[BufferIndex].Status.HasGrabbed = true;
		FrameBuffer[BufferIndex].CaptureTick = getCPUTickCount();
		RegisterNoError();
	}
	else
	{
		cerr << "Failed to grab frame for camera " << Settings.DeviceInfo.device_description << " with buffer " << BufferIndex <<endl;
		FrameBuffer[BufferIndex].Status = BufferStatus();
		RegisterError();
	}
	
	return grabsuccess;
}

bool VideoCaptureCamera::Read(int BufferIndex)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	if (!connected)
	{
		return false;
	}
	bool ReadSuccess = false;
	buff.FrameRaw.HasCPU = false; 
	#ifdef WITH_CUDA
	buff.FrameRaw.HasGPU = false;
	#endif
	if (buff.Status.HasGrabbed)
	{
		ReadSuccess = feed->retrieve(buff.FrameRaw.CPUFrame);
	}
	else
	{
		ReadSuccess = feed->read(buff.FrameRaw.CPUFrame);
		buff.CaptureTick = getCPUTickCount();
	}
	buff.FrameRaw.HasCPU = ReadSuccess;
	
	if (ReadSuccess)
	{
		buff.Status = BufferStatus();
	}
	else
	{
		cerr << "Failed to read frame for camera " << Settings.DeviceInfo.device_description << " with buffer " << BufferIndex <<endl;
		buff.Status = BufferStatus();
		buff.FrameRaw = MixedFrame();
		buff.FrameUndistorted = MixedFrame();
		buff.GrayFrame = MixedFrame();
		buff.RescaledFrame = MixedFrame();
		RegisterError();
		return false;
	}
	RegisterNoError();
	return true;
}

bool VideoCaptureCamera::InjectImage(int BufferIndex, UMat& frame)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	buff.FrameRaw.CPUFrame = frame;
	buff.FrameRaw.HasCPU = true;
	#ifdef WITH_CUDA
	buff.FrameRaw.HasGPU = false;
	#endif
	buff.Status = BufferStatus();
	return true;
}