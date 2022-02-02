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
	GrabbedCPU,
	GrabbedGPU,
	ReadCPU,
	ReadGPU,
	ReadBoth
};

class BufferedFrame
{

protected:
	UMat CPUFrame;
	cuda::GpuMat GPUFrame;
	CameraStatus Status;
	bool HasAruco;

	vector<int> markerIDs;
	vector<vector<Point2f>> markerCorners;

public:

	BufferedFrame()
		:Status(CameraStatus::None)
	{}

	bool GetCPUFrame(UMat& OutFrame);

	bool GetGPUFrame(cuda::GpuMat& OutFrame);

friend class Camera;
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

	bool CudaCapture;

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

	Camera(String InName, Size InCaptureSize, int InFPS, string InDevicePath, int InApiId, bool InCudaCapture = false)
		:Name(InName),
		CaptureSize(InCaptureSize),
		fps(InFPS),
		DevicePath(InDevicePath),
		ApiID(InApiId),
		CudaCapture(InCudaCapture),
		connected(false),
		FrameBuffer(),
		OutputImage()
	{
	}

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

	CameraStatus GetStatus(int BufferIndex);

	bool StartFeed();

	bool Grab(int BufferIndex);

	bool Read(int BufferIndex);

	void detectMarkers(int BufferIndex, Ptr<aruco::Dictionary> dict, Ptr<aruco::DetectorParameters> params);

	virtual void GetFrame(int BufferIndex, UMat& frame) override;

	virtual void GetOutputFrame(int BufferIndex, UMat& frame, Size winsize) override;

};

vector<Camera*> autoDetectCameras();

bool StartCameras(vector<Camera*> Cameras);