#include "data/senders/Encoders/GenericEncoder.hpp"

#include "opencv2/core.hpp"

GenericEncoder::GenericEncoder()
{
    CreationTick = cv::getTickCount();
}

int64 GenericEncoder::GetTick()
{
    return cv::getTickCount() - CreationTick;
}