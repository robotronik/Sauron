#pragma once

#include <opencv2/core.hpp>
#include <opencv2/cudacodec.hpp>

#include "thirdparty/list-devices.hpp"

using namespace cv;
using namespace std;

struct CameraView;

//Progression status for a buffer
struct BufferStatus
{
    bool HasGrabbed;
	bool HasCaptured;
	bool HasUndistorted;
	bool HasResized;
	bool HasAruco;
	bool HasViews;

	BufferStatus()
	:HasGrabbed(false), HasCaptured(false), HasUndistorted(false), HasResized(false), HasAruco(false), HasViews(false)
	{}
};

//Camera API and configuration selection
enum class CameraStartType
{
	ANY,
	GSTREAMER_CPU,
	GSTREAMER_NVDEC,
	GSTREAMER_JETSON,
	GSTREAMER_NVARGUS,
	CUDA
};

//All the settings needed to start a camera, in the palm of your hand...
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
	v4l2::devices::DEVICE_INFO DeviceInfo;

	//Initialisation string
	String StartPath;
	//API to be used for opening
	int ApiID;

	//FOV and center
	Mat CameraMatrix;
	//Distortion
	Mat distanceCoeffs;

	CameraSettings()
	:Resolution(-1,-1), Framerate(0), FramerateDivider(1),
	BufferSize(0), ApiID(-1)
	{}

	bool IsValid();
};

//class that can represent either a GPU-side frame or a CPU-side frame, with helper functions to convert between both sides
class MixedFrame
{

public:
	UMat CPUFrame;
	cuda::GpuMat GPUFrame;
	//Is the CPUFrame valid ?
	bool HasCPU;
	//Is the GPU frame valid ?
	bool HasGPU;

	MixedFrame()
	:HasCPU(false),
	HasGPU(false)
	{}

	MixedFrame(UMat& InCPUFrame)
	:CPUFrame(InCPUFrame),
	HasCPU(true),
	HasGPU(false)
	{}

	MixedFrame(cuda::GpuMat& InGPUFrame)
	:GPUFrame(InGPUFrame),
	HasGPU(true),
	HasCPU(false)
	{}

	//Are any of the frames valid ?
	bool IsValid();

	//Return the size of the first valid stored frame
	Size GetSize();

	//Avoid using because it causes copies
	//Make the frame available on the CPU side then copy into memory
	bool GetCPUFrame(UMat& frame);
	//Avoid using because it causes copies
	//Make the frame available on the GPU suide then copy into memory
	bool GetGPUFrame(cuda::GpuMat& frame);

	//If the frame is available on the CPU do nothing
	//If it's available on the GPU, download it
	//If not we're fucked
	bool MakeCPUAvailable();

	//If the frame is available on the GPU do nothing
	//If it's available on the CPU, upload it
	//If not we're fucked
	bool MakeGPUAvailable();

};

class ObjectTracker;

class BufferedFrame
{

public:
	MixedFrame FrameRaw;
	MixedFrame FrameUndistorted;
	BufferStatus Status;

	vector<MixedFrame> rescaledFrames;

	vector<int> markerIDs;
	vector<vector<Point2f>> markerCorners;
	vector<CameraView> markerViews;

public:

	BufferedFrame()
		:Status()
	{}

	bool GetFrameRaw(MixedFrame& OutFrame);

	bool GetFrameUndistorted(MixedFrame& OutFrame);

	bool GetRescaledFrame(int index, MixedFrame& OutFrame);

};