#pragma once

#include <opencv2/core.hpp>

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
