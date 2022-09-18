#pragma once

#include <opencv2/core.hpp>


//Father class to represent any object capable of outputting a 2D image
class OutputImage
{

public:
	OutputImage(/* args */);
	~OutputImage();

	virtual void SetFrame(int BufferIndex, cv::UMat& frame);

	virtual void GetFrame(int BufferIndex, cv::UMat& frame);

	virtual void GetOutputFrame(int BufferIndex, cv::UMat& frame, cv::Size winsize);
};

cv::UMat ConcatCameras(int BufferIndex, std::vector<OutputImage*> Cameras, int NumCams);
