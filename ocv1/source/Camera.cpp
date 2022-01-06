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
		string capname = string("/dev/video") + to_string(i);
		VideoCapture cap(capname);
		if (cap.isOpened())
		{
			cout << "Found camera at " << i << endl;
			Camera* cam = new Camera("Camera" + to_string(i), Size(1920, 1080), 30, capname, CAP_ANY);
			i++;
			//cam->StartFeed();
			readCameraParameters("c920", cam->CameraMatrix, cam->distanceCoeffs);
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