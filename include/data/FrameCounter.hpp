#pragma once

#include <opencv2/core.hpp>

class FrameCounter
{
private:
    int64 dt;
public:
    FrameCounter(/* args */);
    ~FrameCounter();

    // Returns the time elapsed since last call of this function
    double GetDeltaTime();

    void AddFpsToImage(cv::InputOutputArray img, double DeltaTime);
};
