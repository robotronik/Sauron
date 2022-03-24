#pragma once

#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

class OutputImage
{

public:
	OutputImage(/* args */);
	~OutputImage();

	virtual void SetFrame(int BufferIndex, UMat& frame);

	virtual void GetFrame(int BufferIndex, UMat& frame);

	virtual void GetOutputFrame(int BufferIndex, UMat& frame, Size winsize);
};

UMat ConcatCameras(int BufferIndex, vector<OutputImage*> Cameras, int NumCams);
