#include "data/senders/Encoders/GenericEncoder.hpp"

#include "opencv2/core.hpp"

GenericEncoder::GenericEncoder(uint8_t InAllowMask)
    :AllowMask(InAllowMask)
{
    CreationTick = cv::getTickCount();
}

int64 GenericEncoder::GetTick()
{
    return cv::getTickCount() - CreationTick;
}