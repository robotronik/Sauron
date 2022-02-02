#pragma once

#include <iostream>
#include <string>   // for strings
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudawarping.hpp>

#include "OutputImage.hpp"

using namespace std;
using namespace cv;

enum class CameraStatus
{
	None,
	Grabbed,
	Read,
	Arucoed
};

class Camera : public OutputImage
{

private:
	//config
	String Name;
	Size CaptureSize;
	int fps;
	string DevicePath;
	int ApiID;

	bool CudaCapture;

	//capture
	VideoCapture* feed;
	UMat frame;

	Ptr<cudacodec::VideoReader> d_reader;
	cuda::GpuMat d_frame;

public:
	//calibration
	Mat CameraMatrix;
	Mat distanceCoeffs;

public:
	//status
	bool connected;
	CameraStatus ReadStatus;

public:
	//aruco
	vector<int> markerIDs;
	vector<vector<Point2f>> markerCorners;

public:

	Camera(String InName, Size InCaptureSize, int InFPS, string InDevicePath, int InApiId, bool InCudaCapture = false)
		:Name(InName),
		CaptureSize(InCaptureSize),
		fps(InFPS),
		DevicePath(InDevicePath),
		ApiID(InApiId),
		CudaCapture(InCudaCapture),
		connected(false),
		ReadStatus(CameraStatus::None),
		OutputImage()
	{}

	~Camera()
	{
		if (connected)
		{
			if (CudaCapture)
			{
				delete d_reader;
			}
			else
			{
				delete feed;
			}
			
			
		}
		
	}

	String GetName();

	CameraStatus GetStatus();

	bool StartFeed();

	bool Grab();

	bool Read();

	void detectMarkers(Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params);

	virtual void GetFrame(UMat& frame) override;

	virtual void GetOutputFrame(UMat& frame, Size winsize) override;

};

vector<Camera*> autoDetectCameras();

bool StartCameras(vector<Camera*> Cameras);