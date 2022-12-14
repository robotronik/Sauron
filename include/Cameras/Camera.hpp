#pragma once


#include <iostream>
#include <string>   // for strings
#include <vector>
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>

#ifdef WITH_CUDA
#include <opencv2/cudacodec.hpp>
#endif

#include "data/OutputImage.hpp"
#include "data/ImageTypes.hpp"
#include "TrackedObjects/TrackedObject.hpp"

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


class Camera : public OutputImage, public TrackedObject
{
protected:
	//config
	CameraSettings Settings;

	bool HasUndistortionMaps;
	
	#ifdef WITH_CUDA
	cv::cuda::GpuMat UndistMap1, UndistMap2;
	#else
	cv::UMat UndistMap1, UndistMap2;
	#endif
public:
	int errors;

protected:
	//frame buffer, increases fps but also latency
	std::vector<BufferedFrame> FrameBuffer;

public:
	//status
	bool connected;

public:

	Camera(CameraSettings InSettings)
		:TrackedObject(), Settings(InSettings),
		HasUndistortionMaps(false),
		errors(0),
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
	std::vector<std::vector<cv::Point2f>> imagePoints, std::vector<std::string> imagePaths, cv::Size imageSize,
	cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs);

	virtual void GetFrame(int BufferIndex, cv::UMat& frame) override;

	virtual void GetOutputFrame(int BufferIndex, cv::UMat& frame, cv::Rect window) override;

	virtual std::vector<ObjectData> ToObjectData(int BaseNumeral) override;

	virtual cv::Affine3d GetObjectTransform(const CameraArucoData& CameraData, float& Surface, float& ReprojectionError) override;
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

	void SetMarkerReprojection(int MarkerIndex, const std::vector<cv::Point2d> &Corners);

	//convert corner pixel location into 3D space relative to camera
	//Arco tags must be registered into the registry beforehand to have correct depth
	virtual void SolveMarkers(int BufferIndex, int CameraIdx, ObjectTracker* registry);

	virtual int GetCameraViewsSize(int BufferIndex);

	//Gather detected markers in 3D space relative to the camera
	virtual bool GetCameraViews(int BufferIndex, std::vector<CameraView>& views);

};