#pragma once

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>


cv::Ptr<cv::aruco::Dictionary> GetArucoDict();

cv::Ptr<cv::aruco::DetectorParameters> GetArucoParams();

cv::Size GetScreenSize();

cv::Size GetFrameSize();

int GetCaptureFramerate();

//list of downscales to be done to the aruco detections
std::vector<float> GetReductionFactor();

//list of resolutions in the end
std::vector<cv::Size> GetArucoReductions();

cv::UMat& GetArucoImage(int id);