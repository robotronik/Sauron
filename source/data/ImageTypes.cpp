#include "data/ImageTypes.hpp"
#include "data/CameraView.hpp"

using namespace cv;
using namespace std;

bool CameraSettings::IsValid()
{
	return Framerate >0 && Resolution.width >0 && Resolution.height >0 && BufferSize > 0 && FramerateDivider > 0;
}

bool MixedFrame::IsValid()
{
	return HasCPU 
	#ifdef WITH_CUDA 
	|| HasGPU
	#endif
	;
}

Size MixedFrame::GetSize()
{
	if (HasCPU)
	{
		return CPUFrame.size();
	}
	#ifdef WITH_CUDA
	if (HasGPU)
	{
		return GPUFrame.size();
	}
	#endif
	return Size();
}

bool MixedFrame::GetCPUFrame(UMat& frame)
{
	if (!MakeCPUAvailable())
	{
		return false;
	}
	frame = CPUFrame;
	return true;
}

#ifdef WITH_CUDA
bool MixedFrame::GetGPUFrame(cuda::GpuMat& frame)
{
	if (!MakeGPUAvailable())
	{
		return false;
	}
	frame = GPUFrame;
	return true;
}
#endif

bool MixedFrame::MakeCPUAvailable()
{
	if (!HasCPU)
	{
		#ifdef WITH_CUDA
		if (!HasGPU)
		{
			return false;
		}
		GPUFrame.download(CPUFrame);
		HasCPU = true;
		#else
		return false;
		#endif
	}
	return true;
}

#ifdef WITH_CUDA
bool MixedFrame::MakeGPUAvailable()
{
	if (!HasGPU)
	{
		if (!HasCPU)
		{
			return false;
		}
		
		GPUFrame.upload(CPUFrame);
		HasGPU = true;
	}
	return true;
}
#endif
