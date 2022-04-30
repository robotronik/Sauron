#pragma once

#include <iostream>
#include <string>   // for strings
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>

#include <opencv2/cudacodec.hpp>

#include "data/OutputImage.hpp"
#include "data/ImageTypes.hpp"

using namespace std;
using namespace cv;

class Camera : public OutputImage
{
protected:
	//config
	CameraSettings Settings;

	bool HasUndistortionMaps;
	cuda::GpuMat UndistMap1, UndistMap2;

public:
	Affine3d Location;

private:
	//capture using classic api
	VideoCapture* feed;

	//capture using cuda
	Ptr<cudacodec::VideoReader> d_reader;
protected:
	//frame buffer, increases fps but also latency
	vector<BufferedFrame> FrameBuffer;

public:
	//calibration

public:
	//status
	bool connected;

public:

	Camera(CameraSettings InSettings)
		:Settings(InSettings),
		Location(Affine3d::Identity()),
		connected(false),
		FrameBuffer(),
		OutputImage()
	{
	}

	~Camera()
	{
		if (connected)
		{
			if (Settings.StartType == CameraStartType::CUDA)
			{
				delete d_reader;
			}
			else
			{
				delete feed;
			}
			
			
		}
		
	}

	//Get the settings used to start this camera
	virtual CameraSettings GetCameraSettings();

	//Set the settings to used for this camera
	virtual bool SetCameraSetting(CameraSettings InSettings);

	virtual bool SetCalibrationSetting(Mat CameraMatrix, Mat DistanceCoefficients);

	//Get the status of a buffer (read, aruco'ed, 3D-ed...)
	virtual BufferStatus GetStatus(int BufferIndex);

	//Start the camera
	virtual bool StartFeed();

	//Lock a frame to be capture at this time
	//This allow for simultaneous capture
	virtual bool Grab(int BufferIndex);

	//Retrieve or read a frame
	virtual bool Read(int BufferIndex);

	virtual bool InjectImage(int BufferIndex, UMat& frame);

	virtual void GetFrame(int BufferIndex, UMat& frame) override;

	virtual void Undistort(int BufferIdx);

	virtual void GetFrameUndistorted(int BufferIndex, UMat& frame);

	//Create lower-resolution copies of the frame to be used in aruco detection
	virtual void RescaleFrames(int BufferIdx);

	//detect markers using the lower resolutions and improve accuracy using the higher-resolution image
	virtual void detectMarkers(int BufferIndex, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params);

	//Gather detected markers in screen space
	virtual bool GetMarkerData(int BufferIndex, vector<int>& markerIDs, vector<vector<Point2f>>& markerCorners);

	//convert corner pixel location into 3D space relative to camera
	//Arco tags must be registered into the registry beforehand to have correct depth
	virtual void SolveMarkers(int BufferIndex, int CameraIdx, ObjectTracker* registry);

	virtual int GetCameraViewsSize(int BufferIndex);

	//Gather detected markers in 3D space relative to the camera
	virtual bool GetCameraViews(int BufferIndex, vector<CameraView>& views);

	//Get frame with marker data and everything for the required resolution
	virtual void GetOutputFrame(int BufferIndex, UMat& frame, Size winsize) override;

	virtual void Calibrate(vector<vector<Point3f>> objectPoints,
	vector<vector<Point2f>> imagePoints, Size imageSize,
	Mat& cameraMatrix, Mat& distCoeffs,
	OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs);

};


vector<CameraSettings> autoDetectCameras(CameraStartType Start, String Filter, String CalibrationFile, bool silent = true);

template<class CameraClass>
vector<CameraClass*> StartCameras(vector<CameraSettings> CameraSettings)
{
	vector<CameraClass*> Cameras;
	for (int i = 0; i < CameraSettings.size(); i++)
	{
		Camera* cam = new CameraClass(CameraSettings[i]);
		Cameras.push_back(cam);
		if(!Cameras[i]->StartFeed())
		{
			cerr << "ERROR! Unable to open camera " << Cameras[i]->GetCameraSettings().DeviceInfo.device_description;
		}
	}
	return Cameras;
}