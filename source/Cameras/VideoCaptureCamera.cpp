#include "Cameras/VideoCaptureCamera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/imgproc.hpp>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>

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

using namespace std;
using namespace v4l2::devices;

bool VideoCaptureCamera::StartFeed()
{
	if (connected)
	{
		return false;
	}
	FrameBuffer.resize(Settings.BufferSize);
	for (int i = 0; i < Settings.BufferSize; i++)
	{
		FrameBuffer[i] = BufferedFrame();
	}
	
	if (Settings.StartType == CameraStartType::CUDA)
	{
		Settings.StartPath = Settings.DeviceInfo.device_paths[0];
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
		switch (Settings.StartType)
		{
		case CameraStartType::GSTREAMER_NVARGUS:
			{
				//Settings.StartPath = "nvarguscamerasrc ! nvvidconv ! videoconvert ! appsink drop=1";
				ostringstream capnamestream;
				capnamestream << "nvarguscamerasrc ! video/x-raw(memory:NVMM), " 
				<< sizestream.str() << ", framerate=(fraction)"
				<< (int)Settings.Framerate << "/" << (int)Settings.FramerateDivider 
				<< " !  nvvidconv ! video/x-raw, format=(string)BGRx, " << sizestream.str() << " ! videoconvert ! video/x-raw, format=BGR ! appsink drop=1";
				Settings.StartPath = capnamestream.str();
				Settings.ApiID = CAP_GSTREAMER;
			}
			break;
		case CameraStartType::GSTREAMER_CPU:
		case CameraStartType::GSTREAMER_NVDEC:
		case CameraStartType::GSTREAMER_JETSON:
			{
				ostringstream capnamestream;
				capnamestream << "v4l2src device=" << Settings.DeviceInfo.device_paths[0] << " io-mode=4 ! image/jpeg, width=" 
				<< Settings.Resolution.width << ", height=" << Settings.Resolution.height << ", framerate="
				<< (int)Settings.Framerate << "/" << (int)Settings.FramerateDivider << ", num-buffers=1 ! ";
				if (Settings.StartType == CameraStartType::GSTREAMER_CPU)
				{
					capnamestream << "jpegdec ! videoconvert ! ";
				}
				else if (Settings.StartType == CameraStartType::GSTREAMER_JETSON)
				{
					capnamestream << "nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! ";
				}
				else if(Settings.StartType == CameraStartType::CUDA)
				{
					capnamestream << "nvdec ! glcolorconvert ! gldownload ! ";
				}
				capnamestream << "video/x-raw, format=BGR ! appsink";
				Settings.StartPath = capnamestream.str();
				Settings.ApiID = CAP_GSTREAMER;
			}
			break;
		default:
			cerr << "WARNING : Unrecognised Camera Start Type in VideoCaptureCamera, defaulting to auto API" << endl;
		case CameraStartType::ANY:
			Settings.StartPath = Settings.DeviceInfo.device_paths[0];
			Settings.ApiID = CAP_ANY;
			break;
		}

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
			feed->set(CAP_PROP_BUFFERSIZE, 1);
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
	}
	else
	{
		cerr << "Failed to grab frame for camera " << Settings.DeviceInfo.device_description << " with buffer " << BufferIndex <<endl;
		FrameBuffer[BufferIndex].Status = BufferStatus();
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
	buff.FrameRaw.HasCPU = false; buff.FrameRaw.HasGPU = false;
	if (buff.Status.HasGrabbed)
	{
		ReadSuccess = feed->retrieve(buff.FrameRaw.CPUFrame);
	}
	else
	{
		ReadSuccess = feed->read(buff.FrameRaw.CPUFrame);
	}
	buff.FrameRaw.HasCPU = ReadSuccess;
	
	if (ReadSuccess)
	{
		buff.Status = BufferStatus();
		buff.Status.HasCaptured = true;
	}
	else
	{
		cerr << "Failed to read frame for camera " << Settings.DeviceInfo.device_description << " with buffer " << BufferIndex <<endl;
		buff.Status = BufferStatus();
		buff.FrameRaw = MixedFrame();
		buff.FrameUndistorted = MixedFrame();
		buff.rescaledFrames.resize(0);
		return false;
	}
	
	return true;
}

bool VideoCaptureCamera::InjectImage(int BufferIndex, UMat& frame)
{
	BufferedFrame& buff = FrameBuffer[BufferIndex];
	buff.FrameRaw.CPUFrame = frame;
	buff.FrameRaw.HasCPU = true;
	buff.FrameRaw.HasGPU = false;
	buff.Status = BufferStatus();
	buff.Status.HasCaptured = true;
	return true;
}