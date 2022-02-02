#pragma once

#include <opencv2/core.hpp>

using namespace cv;

class OutputImage
{

public:
    OutputImage(/* args */);
    ~OutputImage();

    virtual void SetFrame(UMat& frame);

    virtual void GetFrame(UMat& frame);

    virtual void GetOutputFrame(UMat& frame, Size winsize);
};
