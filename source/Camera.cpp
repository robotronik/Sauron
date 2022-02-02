#include "Camera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/imgproc.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "list-devices.hpp"
#include "Calibfile.hpp"

using namespace std;

bool BufferedFrame::GetCPUFrame(UMat& OutFrame)
{
	switch (Status)
	{
	case CameraStatus::ReadGPU :
		GPUFrame.download(CPUFrame);
		Status = CameraStatus::ReadBoth;
	case CameraStatus::ReadCPU :
	case CameraStatus::ReadBoth :
		OutFrame = CPUFrame;
		return true;
	default:
		return false;
		break;
	}
}

bool BufferedFrame::GetGPUFrame(cuda::GpuMat& OutFrame)
{
	switch (Status)
	{
	case CameraStatus::ReadCPU :
		GPUFrame.upload(CPUFrame);
		Status = CameraStatus::ReadBoth;
	case CameraStatus::ReadGPU :
	case CameraStatus::ReadBoth :
		OutFrame = GPUFrame;
		return true;
	default:
		return false;
		break;
	}
}

String Camera::GetName()
{
	return Name;
}

CameraStatus Camera::GetStatus(int BufferIndex)
{
	return FrameBuffer[BufferIndex].Status;
}

bool Camera::StartFeed()
{
	if (connected)
	{
		return false;
	}
	for (int i = 0; i < FrameBufferSize; i++)
	{
		FrameBuffer[i] = BufferedFrame();
	}
	
	if (CudaCapture)
	{
		d_reader = cudacodec::createVideoReader(DevicePath, {
			CAP_PROP_FRAME_WIDTH, CaptureSize.width, 
			CAP_PROP_FRAME_HEIGHT, CaptureSize.height, 
			CAP_PROP_FPS, fps,
			CAP_PROP_BUFFERSIZE, 1});

	}
	else
	{
		feed = new VideoCapture();
		cout << "Opening device at \"" << DevicePath << "\" with API id " << ApiID << endl;
		feed->open(DevicePath, ApiID);
		feed->set(CAP_PROP_FRAME_WIDTH, CaptureSize.width);
		feed->set(CAP_PROP_FRAME_HEIGHT, CaptureSize.height);
		feed->set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
		feed->set(CAP_PROP_FPS, fps);
		feed->set(CAP_PROP_BUFFERSIZE, 1);
	}
	
	connected = true;
	return true;
}

bool Camera::Grab(int BufferIndex)
{
	if (!connected)
	{
		return false;
	}
	bool grabsuccess = false;
	if (CudaCapture)
	{
		grabsuccess = d_reader->grab();
	}
	else
	{
		grabsuccess = feed->grab();
	}
	if (grabsuccess)
	{
		FrameBuffer[BufferIndex].Status = CudaCapture ? CameraStatus::GrabbedGPU : CameraStatus::GrabbedCPU;
	}
	else
	{
		cerr << "Failed to grab frame for camera " << Name << " with buffer " << BufferIndex <<endl;
		FrameBuffer[BufferIndex].Status = CameraStatus::None;
	}
	
	return grabsuccess;
}

bool Camera::Read(int BufferIndex)
{
	if (!connected)
	{
		return false;
	}
	bool ReadSuccess = false;
	if (CudaCapture)
	{
		if (FrameBuffer[BufferIndex].Status == CameraStatus::GrabbedGPU)
		{
			ReadSuccess = d_reader->retrieve(FrameBuffer[BufferIndex].GPUFrame);
		}
		else
		{
			ReadSuccess = d_reader->nextFrame(FrameBuffer[BufferIndex].GPUFrame);
		}
	}
	else
	{
		if (FrameBuffer[BufferIndex].Status == CameraStatus::GrabbedCPU)
		{
			ReadSuccess = feed->retrieve(FrameBuffer[BufferIndex].CPUFrame);
		}
		else
		{
			ReadSuccess = feed->read(FrameBuffer[BufferIndex].CPUFrame);
		}
	}
	if (ReadSuccess)
	{
		FrameBuffer[BufferIndex].Status = CudaCapture ? CameraStatus::ReadGPU : CameraStatus::ReadCPU;
		FrameBuffer[BufferIndex].HasAruco = false;
	}
	else
	{
		cerr << "Failed to read frame for camera " << Name << " with buffer " << BufferIndex <<endl;
		FrameBuffer[BufferIndex].Status = CameraStatus::None;
	}
	return ReadSuccess;
}

void Camera::GetFrame(int BufferIndex, UMat& OutFrame)
{
	FrameBuffer[BufferIndex].GetCPUFrame(OutFrame);
}

void Camera::GetOutputFrame(int BufferIndex, UMat& OutFrame, Size winsize)
{
	cuda::GpuMat resizedgpu, framegpu;
	FrameBuffer[BufferIndex].GetGPUFrame(framegpu);

	double fx = framegpu.cols / winsize.width;
	double fy = framegpu.rows / winsize.height;
	double fz = max(fx, fy);

	cuda::resize(framegpu, resizedgpu, Size(winsize.width, winsize.height), fz, fz, INTER_LINEAR);
	resizedgpu.download(OutFrame);
	//cout << "Resize OK" <<endl;
	if (FrameBuffer[BufferIndex].HasAruco)
	{
		vector<vector<Point2f>> raruco;
		for (int i = 0; i < FrameBuffer[BufferIndex].markerCorners.size(); i++)
		{
			vector<Point2f> marker;
			for (int j = 0; j < FrameBuffer[BufferIndex].markerCorners[i].size(); j++)
			{
				marker.push_back(FrameBuffer[BufferIndex].markerCorners[i][j]/fz);
			}
			raruco.push_back(marker);
		}
		aruco::drawDetectedMarkers(OutFrame, raruco, FrameBuffer[BufferIndex].markerIDs);
	}
}

void Camera::detectMarkers(int BufferIndex, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	/*cuda::GpuMat framegpu; framegpu.upload(frame);
	cuda::cvtColor(framegpu, framegpu, COLOR_BGR2GRAY);
	cuda::threshold(framegpu, framegpu, 127, 255, THRESH_BINARY);
	UMat framethreshed; framegpu.download(framethreshed);*/
	UMat FrameCPU;

	if(!FrameBuffer[BufferIndex].GetCPUFrame(FrameCPU))
	{
		return;
	}
	aruco::detectMarkers(FrameCPU, dict, FrameBuffer[BufferIndex].markerCorners, FrameBuffer[BufferIndex].markerIDs, params);
	FrameBuffer[BufferIndex].HasAruco = true;
}

vector<Camera*> autoDetectCameras()
{
	vector<v4l2::devices::DEVICE_INFO> devices;

    v4l2::devices::list(devices);

    for (const auto & device : devices) 
    {
    
        cout << device.device_description <<  " at " << device.bus_info << " is attached to\n";

        for (const auto & path : device.device_paths) {
            cout << path << "\n";
        }
        
    }

    vector<Camera*> detected;
	for (const auto & device : devices)
	{
		//v4l2-ctl --list-formats-ext
		//gst-launch-1.0 v4l2src device="/dev/video0" io-mode=2 ! "image/jpeg, width=1920, height=1080, framerate=30/1" ! nvjpegdec ! "video/x-raw" ! nvoverlaysink -e
		//gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-h264, format=H264, width=1920, height=1080, framerate=30/1" ! h264parse ! omxh264dec ! nvvidconv ! "video/x-raw(memory:NVMM), format=NV12" ! nvoverlaysink -e
		//nvv4l2decoder ?
		//string capname = string("v4l2src device=/dev/video") + to_string(i)
		//+ String(" io-mode=2 do-timestamp=true ! image/jpeg, width=1920, height=1080, framerate=30/2 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink");
		
		if (strstr(device.device_description.c_str(), "C920") != NULL)
		{
			string capname = String(device.device_paths[0]);
			int API = CAP_ANY;
			Camera* cam = new Camera(device.device_description, Size(1920, 1080), 30, capname, API, false);
			//cam->StartFeed();
			readCameraParameters(String("../calibration/")+device.device_description, cam->CameraMatrix, cam->distanceCoeffs);
			detected.push_back(cam);
		}
	}
    return detected;
}

bool StartCameras(vector<Camera*> Cameras)
{
	for (int i = 0; i < Cameras.size(); i++)
	{
		if(!Cameras[i]->StartFeed())
		{
			cout << "ERROR! Unable to open camera " << Cameras[i]->GetName();
		}
	}
	return true;
}