#include "data/ImageTypes.hpp"

bool MixedFrame::IsValid()
{
	return HasCPU || HasGPU;
}

bool MixedFrame::GetCPUFrame(UMat& frame)
{

	if (!HasCPU)
	{
		if (!HasGPU)
		{
			return false;
		}
		GPUFrame.download(CPUFrame);
		HasCPU = true;
	}
	frame = CPUFrame;
	return true;
}

bool MixedFrame::GetGPUFrame(cuda::GpuMat& frame)
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
	frame = GPUFrame;
	return true;
}

bool BufferedFrame::GetFrameRaw(MixedFrame& OutFrame)
{
	OutFrame = FrameRaw;
	return FrameRaw.IsValid();
}

bool BufferedFrame::GetFrameUndistorted(MixedFrame& OutFrame)
{
	OutFrame = FrameUndistorted;
	return FrameUndistorted.IsValid();
}

bool BufferedFrame::GetRescaledFrame(int index, MixedFrame& OutFrame)
{
	OutFrame = rescaledFrames[index];
	return rescaledFrames[index].IsValid();
}