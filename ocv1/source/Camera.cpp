#include "Camera.hpp"


#include <iostream> // for standard I/O
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/imgproc.hpp>

vector<Camera*> autoDetectCameras(int maxidx)
{
    vector<Camera*> detected;
	for (int i = 0; i < maxidx; i++)
	{
		//v4l2-ctl --list-formats-ext
		//gst-launch-1.0 v4l2src device="/dev/video0" io-mode=2 ! "image/jpeg, width=1920, height=1080, framerate=30/1" ! nvjpegdec ! "video/x-raw" ! nvoverlaysink -e
		//gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-h264, format=H264, width=1920, height=1080, framerate=30/1" ! h264parse ! omxh264dec ! nvvidconv ! "video/x-raw(memory:NVMM), format=NV12" ! nvoverlaysink -e
		//nvv4l2decoder ?
		string capname = string("v4l2src device=/dev/video") + to_string(i)
		+ String(" io-mode=2 do-timestamp=true ! image/jpeg, width=1920, height=1080, framerate=30/2 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink");
		int API = CAP_GSTREAMER;
		VideoCapture* cap = new VideoCapture(capname, API);
		if (cap->isOpened())
		{
			cout << "Found camera at " << i << endl;
			Camera* cam = new Camera("Camera" + to_string(i), Size(1920, 1080), 30, capname, API);
			//cam->StartFeed();
			readCameraParameters("c920", cam->CameraMatrix, cam->distanceCoeffs);
			detected.push_back(cam);
		}
		delete cap;
	}
    return detected;
}

bool StartCameras(vector<Camera*> Cameras)
{
	for (int i = 0; i < Cameras.size(); i++)
	{
		if(!Cameras[i]->StartFeed())
		{
			cout << "ERROR! Unable to open camera " << Cameras[i]->WindowName;
		}
	}
	return true;
}

static bool readCameraParameters(std::string filename, Mat& camMatrix, Mat& distCoeffs)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return (camMatrix.size() == cv::Size(3,3)) ;
}