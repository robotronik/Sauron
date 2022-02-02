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

String Camera::GetName()
{
	return Name;
}

CameraStatus Camera::GetStatus()
{
	return ReadStatus;
}

bool Camera::StartFeed()
{
	if (connected)
	{
		return false;
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

bool Camera::Grab()
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
		ReadStatus = CameraStatus::Grabbed;
	}
	else
	{
		ReadStatus = CameraStatus::None;
	}
	
	return grabsuccess;
}

bool Camera::Read()
{
	if (!connected)
	{
		return false;
	}
	bool ReadSuccess = false;
	if (CudaCapture)
	{
		if (ReadStatus == CameraStatus::Grabbed)
		{
			ReadSuccess = d_reader->retrieve(d_frame);
		}
		else
		{
			ReadSuccess = d_reader->nextFrame(d_frame);
		}
		d_frame.download(frame);
	}
	else
	{
		if (ReadStatus == CameraStatus::Grabbed)
		{
			ReadSuccess = feed->retrieve(frame);
		}
		else
		{
			ReadSuccess = feed->read(frame);
		}
	}
	if (ReadSuccess)
	{
		ReadStatus = CameraStatus::Read;
	}
	else
	{
		ReadStatus = CameraStatus::None;
	}
	return ReadSuccess;
}

void Camera::GetFrame(UMat& OutFrame)
{
	OutFrame = frame;
}

void Camera::GetOutputFrame(UMat& OutFrame, Size winsize)
{
	double fx = frame.cols / winsize.width;
	double fy = frame.rows / winsize.height;
	double fz = max(fx, fy);
	cuda::GpuMat resizedgpu, framegpu;
	framegpu.upload(frame);
	cuda::resize(framegpu, resizedgpu, Size(winsize.width, winsize.height), fz, fz, INTER_LINEAR);
	resizedgpu.download(OutFrame);
	//cout << "Resize OK" <<endl;
	if (ReadStatus == CameraStatus::Arucoed)
	{
		vector<vector<Point2f>> raruco;
		for (int i = 0; i < markerCorners.size(); i++)
		{
			vector<Point2f> marker;
			for (int j = 0; j < markerCorners[i].size(); j++)
			{
				marker.push_back(markerCorners[i][j]/fz);
			}
			raruco.push_back(marker);
		}
		aruco::drawDetectedMarkers(OutFrame, raruco, markerIDs);
	}
}

void Camera::detectMarkers(Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params)
{
	if (frame.empty())
	{
		return;
	}
	/*cuda::GpuMat framegpu; framegpu.upload(frame);
	cuda::cvtColor(framegpu, framegpu, COLOR_BGR2GRAY);
	cuda::threshold(framegpu, framegpu, 127, 255, THRESH_BINARY);
	UMat framethreshed; framegpu.download(framethreshed);*/
	aruco::detectMarkers(frame, dict, markerCorners, markerIDs, params);
	ReadStatus = CameraStatus::Arucoed;
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