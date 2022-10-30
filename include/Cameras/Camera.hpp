#pragma once


#include <iostream>
#include <string>   // for strings
#include <vector>
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>

#include <opencv2/cudacodec.hpp>

#include "data/OutputImage.hpp"
#include "data/ImageTypes.hpp"

class Camera;
struct CameraArucoData;

template<class CameraClass>
std::vector<CameraClass*> StartCameras(std::vector<CameraSettings> CameraSettings)
{
	std::vector<CameraClass*> Cameras;
	for (int i = 0; i < CameraSettings.size(); i++)
	{
		Camera* cam = new CameraClass(CameraSettings[i]);
		Cameras.push_back((CameraClass*)cam);
		if(!Cameras[i]->StartFeed())
		{
			std::cerr << "ERROR! Unable to open camera " << Cameras[i]->GetCameraSettings().DeviceInfo.device_description << std::endl;
		}
	}
	return Cameras;
}


class Camera : public OutputImage
{
protected:
	//config
	CameraSettings Settings;

	bool HasUndistortionMaps;
	cv::cuda::GpuMat UndistMap1, UndistMap2;

public:
	int errors;

public:
	cv::Affine3d Location;

protected:
	//frame buffer, increases fps but also latency
	std::vector<BufferedFrame> FrameBuffer;

public:
	//status
	bool connected;

public:

	Camera(CameraSettings InSettings)
		:Settings(InSettings),
		errors(0),
		Location(cv::Affine3d::Identity()),
		connected(false),
		FrameBuffer()
	{
	}

	~Camera()
	{
		
	}

protected:
	void RegisterError();
	void RegisterNoError();
public:

	//Get the settings used to start this camera
	virtual CameraSettings GetCameraSettings();

	//Set the settings to used for this camera
	virtual bool SetCameraSetting(CameraSettings InSettings);

	virtual bool SetCalibrationSetting(cv::Mat CameraMatrix, cv::Mat DistanceCoefficients);

	virtual void GetCameraSettingsAfterUndistortion(cv::Mat& CameraMatrix, cv::Mat& DistanceCoefficients);

	//Get the status of a buffer (read, aruco'ed, 3D-ed...)
	virtual BufferStatus GetStatus(int BufferIndex);

	//Start the camera
	virtual bool StartFeed();

	//Lock a frame to be capture at this time
	//This allow for simultaneous capture
	virtual bool Grab(int BufferIndex);

	//Retrieve or read a frame
	virtual bool Read(int BufferIndex);

	virtual bool InjectImage(int BufferIndex, cv::UMat& frame);

	virtual void Undistort(int BufferIdx);

	virtual void GetFrameUndistorted(int BufferIndex, cv::UMat& frame);

	virtual void Calibrate(std::vector<std::vector<cv::Point3f>> objectPoints,
	std::vector<std::vector<cv::Point2f>> imagePoints, cv::Size imageSize,
	cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	cv::OutputArrayOfArrays rvecs, cv::OutputArrayOfArrays tvecs);

	virtual void GetFrame(int BufferIndex, cv::UMat& frame) override;

	virtual void GetOutputFrame(int BufferIndex, cv::UMat& frame, cv::Size winsize) override;
};

class ArucoCamera : public Camera
{

public:

	ArucoCamera(CameraSettings InCameraSettings)
	:Camera(InCameraSettings)
	{}

	//Create lower-resolution copies of the frame to be used in aruco detection
	virtual void RescaleFrames(int BufferIdx);

	//detect markers using the lower resolutions and improve accuracy using the higher-resolution image
	virtual void detectMarkers(int BufferIndex, cv::Ptr<cv::aruco::Dictionary> dict, cv::Ptr<cv::aruco::DetectorParameters> params);

	//Gather detected markers in screen space
	virtual bool GetMarkerData(int BufferIndex, CameraArucoData& CameraData);

	//convert corner pixel location into 3D space relative to camera
	//Arco tags must be registered into the registry beforehand to have correct depth
	virtual void SolveMarkers(int BufferIndex, int CameraIdx, ObjectTracker* registry);

	virtual int GetCameraViewsSize(int BufferIndex);

	//Gather detected markers in 3D space relative to the camera
	virtual bool GetCameraViews(int BufferIndex, std::vector<CameraView>& views);

};