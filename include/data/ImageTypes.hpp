#pragma once

#include <opencv2/core.hpp>

#ifdef WITH_CUDA
#include <opencv2/cudacodec.hpp>
#endif

#include "thirdparty/list-devices.hpp"
#include "data/CameraView.hpp"


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
	ANY = 0,
	GSTREAMER_CPU = 1,
	GSTREAMER_NVDEC = 2,
	GSTREAMER_JETSON = 3,
	GSTREAMER_NVARGUS = 4,
	CUDA = 5
};

//All the settings needed to start a camera, in the palm of your hand...
struct CameraSettings
{
	//which api should be used ?
	CameraStartType StartType;

	//General data
	//Resolution of the frame to be captured
	cv::Size Resolution;
	//Framerate
	uint8_t Framerate;
	//Framerate divider : you can set 60fps but only sample 1 of 2 frames to have less latency and less computation
	uint8_t FramerateDivider;
	//Size of the camera frame buffer, to pipeline opencv computations
	uint8_t BufferSize;
	//data from v4l2 about the device
	v4l2::devices::DEVICE_INFO DeviceInfo;

	//Initialisation string
	cv::String StartPath;
	//API to be used for opening
	int ApiID;

	//FOV and center
	cv::Mat CameraMatrix;
	//Distortion
	cv::Mat distanceCoeffs;

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
	cv::UMat CPUFrame;
	#ifdef WITH_CUDA
	cv::cuda::GpuMat GPUFrame;
	#endif
	//Is the CPUFrame valid ?
	bool HasCPU;

	#ifdef WITH_CUDA
	//Is the GPU frame valid ?
	bool HasGPU;
	#endif
	MixedFrame()
	:HasCPU(false)
	#ifdef WITH_CUDA
	,HasGPU(false)
	#endif
	{}

	MixedFrame(cv::UMat& InCPUFrame)
	:CPUFrame(InCPUFrame),
	HasCPU(true)
	#ifdef WITH_CUDA
	,HasGPU(false)
	#endif
	{}

	#ifdef WITH_CUDA
	MixedFrame(cv::cuda::GpuMat& InGPUFrame)
	:GPUFrame(InGPUFrame),
	HasGPU(true),
	HasCPU(false)
	{}
	#endif

	//Are any of the frames valid ?
	bool IsValid();

	//Return the size of the first valid stored frame
	cv::Size GetSize();

	//Avoid using because it causes copies
	//Make the frame available on the CPU side then copy into memory
	bool GetCPUFrame(cv::UMat& frame);

	#ifdef WITH_CUDA
	//Avoid using because it causes copies
	//Make the frame available on the GPU suide then copy into memory
	bool GetGPUFrame(cv::cuda::GpuMat& frame);
	#endif

	//If the frame is available on the CPU do nothing
	//If it's available on the GPU, download it
	//If not we're fucked
	bool MakeCPUAvailable();

	#ifdef WITH_CUDA
	//If the frame is available on the GPU do nothing
	//If it's available on the CPU, upload it
	//If not we're fucked
	bool MakeGPUAvailable();
	#endif

};

class ObjectTracker;

class BufferedFrame
{

public:
	unsigned int CaptureTick;
	MixedFrame FrameRaw;
	MixedFrame FrameUndistorted;
	BufferStatus Status;

	std::vector<MixedFrame> rescaledFrames;

	std::vector<int> markerIDs;
	std::vector<std::vector<cv::Point2f>> markerCorners;
	std::vector<CameraView> markerViews;

public:

	BufferedFrame()
		:Status()
	{}

	bool GetFrameRaw(MixedFrame& OutFrame);

	bool GetFrameUndistorted(MixedFrame& OutFrame);

	bool GetRescaledFrame(int index, MixedFrame& OutFrame);

};