#pragma once

#include <iostream>
#include <string>   // for strings
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/core/affine.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudawarping.hpp>

#include "data/OutputImage.hpp"
#include "data/CameraView.hpp"

using namespace std;
using namespace cv;

enum class CameraStatus
{
	None,
	GrabbedCPU,
	GrabbedGPU,
	ReadCPU,
	ReadGPU,
	ReadBoth
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

enum class CameraStartType
{
	ANY,
	GSTREAMER_CPU,
	GSTREAMER_NVDEC,
	CUDA
};

class Camera : public OutputImage
{
public:
	static const int FrameBufferSize = 2;
private:
	//config
	String Name;
	Size CaptureSize;
	int fps;
	string DevicePath;
	int ApiID;
public:
	Affine3d Location;
private:

	CameraStartType CaptureType;

	//capture
	VideoCapture* feed;

	Ptr<cudacodec::VideoReader> d_reader;

	BufferedFrame FrameBuffer[FrameBufferSize];

public:
	//calibration
	Mat CameraMatrix;
	Mat distanceCoeffs;

public:
	//status
	bool connected;

public:

	Camera(String InName, Size InCaptureSize, int InFPS, string InDevicePath, int InApiId, CameraStartType InCaptureType)
		:Name(InName),
		CaptureSize(InCaptureSize),
		fps(InFPS),
		DevicePath(InDevicePath),
		ApiID(InApiId),
		Location(Affine3d::Identity()),
		CaptureType(InCaptureType),
		connected(false),
		FrameBuffer(),
		OutputImage()
	{
	}

	~Camera()
	{
		if (connected)
		{
			if (CaptureType == CameraStartType::CUDA)
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

	CameraStatus GetStatus(int BufferIndex);

	String GetDevicePath();

	bool StartFeed();

	bool Grab(int BufferIndex);

	bool Read(int BufferIndex);

	void detectMarkers(int BufferIndex, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params);

	void SolveMarkers(int BufferIndex, int CameraIdx, ObjectTracker* registry);

	bool GetMarkerData(int BufferIndex, vector<int>& markerIDs, vector<vector<Point2f>>& markerCorners);

	int GetCameraViewsSize(int BufferIndex);

	bool GetCameraViews(int BufferIndex, vector<CameraView>& views);

	virtual void GetFrame(int BufferIndex, UMat& frame) override;

	virtual void GetOutputFrame(int BufferIndex, UMat& frame, Size winsize) override;

};


vector<Camera*> autoDetectCameras(CameraStartType Start, String Filter, String CalibrationFile);

bool StartCameras(vector<Camera*> Cameras);