#include "data/ImageTypes.hpp"
#include "data/CameraView.hpp"

bool CameraSettings::IsValid()
{
	return Framerate >0 && Resolution.width >0 && Resolution.height >0 && BufferSize > 0 && FramerateDivider > 0;
}

bool MixedFrame::IsValid()
{
	return HasCPU || HasGPU;
}

Size MixedFrame::GetSize()
{
	if (HasCPU)
	{
		return CPUFrame.size();
	}
	if (HasGPU)
	{
		return GPUFrame.size();
	}
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

bool MixedFrame::GetGPUFrame(cuda::GpuMat& frame)
{
	if (!MakeGPUAvailable())
	{
		return false;
	}
	frame = GPUFrame;
	return true;
}

bool MixedFrame::MakeCPUAvailable()
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
	return true;
}

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