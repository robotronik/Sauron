#pragma once

#include <iostream>
#include <string>   // for strings
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudawarping.hpp>

#include "thirdparty/list-devices.hpp"

#include "data/OutputImage.hpp"
#include "data/CameraView.hpp"

using namespace std;
using namespace cv;
using namespace v4l2::devices;

enum class CameraStatus
{
	None,
	GrabbedCPU,
	GrabbedGPU,
	ReadCPU,
	ReadGPU,
	ReadBoth
};

enum class CameraStartType
{
	ANY,
	GSTREAMER_CPU,
	GSTREAMER_NVDEC,
	GSTREAMER_JETSON,
	CUDA
};

struct CameraSettings
{
	//which api should be used ?
	CameraStartType StartType;

	//General data
	//Resolution of the frame to be captured
	Size Resolution;
	//Framerate
	uint8_t Framerate;
	//Framerate divider : you can set 60fps but only sample 1 of 2 frames to have less latency and less computation
	uint8_t FramerateDivider;
	//Size of the camera frame buffer, to pipeline opencv computations
	uint8_t BufferSize;
	//data from v4l2 about the device
	DEVICE_INFO DeviceInfo;

	//Initialisation string
	String StartPath;
	//API to be used for opening
	int ApiID;
};


class ObjectTracker;

class BufferedFrame
{

protected:
	UMat CPUFrame;
	cuda::GpuMat GPUFrame;
	CameraStatus Status;
	bool HasAruco;
	bool HasViews;

	vector<UMat> rescaledFrames;

	vector<int> markerIDs;
	vector<vector<Point2f>> markerCorners;
	vector<CameraView> markerViews;

public:

	BufferedFrame()
		:Status(CameraStatus::None)
	{}

	bool GetCPUFrame(UMat& OutFrame);

	bool GetGPUFrame(cuda::GpuMat& OutFrame);

	bool GetRescaledFrame(int index, UMat& OutFrame);

friend class Camera;
};

class Camera : public OutputImage
{
private:
	//config
	CameraSettings Settings;

public:
	Affine3d Location;

private:
	//capture using classic api
	VideoCapture* feed;

	//capture using cuda
	Ptr<cudacodec::VideoReader> d_reader;

	//frame buffer, increases fps but also latency
	vector<BufferedFrame> FrameBuffer;

public:
	//calibration
	Mat CameraMatrix;
	Mat distanceCoeffs;

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

	CameraSettings GetCameraSettings();

	bool SetCameraSetting(CameraSettings InSettings);

	CameraStatus GetStatus(int BufferIndex);

	bool StartFeed();

	bool Grab(int BufferIndex);

	bool Read(int BufferIndex);

	void RescaleFrames(int BufferIdx);

	void detectMarkers(int BufferIndex, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params);

	void SolveMarkers(int BufferIndex, int CameraIdx, ObjectTracker* registry);

	bool GetMarkerData(int BufferIndex, vector<int>& markerIDs, vector<vector<Point2f>>& markerCorners);

	int GetCameraViewsSize(int BufferIndex);

	bool GetCameraViews(int BufferIndex, vector<CameraView>& views);

	virtual void GetFrame(int BufferIndex, UMat& frame) override;

	virtual void GetOutputFrame(int BufferIndex, UMat& frame, Size winsize) override;

};


vector<Camera*> autoDetectCameras(CameraStartType Start, String Filter, String CalibrationFile, bool silent = true);

bool StartCameras(vector<Camera*> Cameras);