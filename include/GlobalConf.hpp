#pragma once

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>

enum class CameraStartType;

struct CaptureConfig
{
    int StartType;
    cv::Size FrameSize;
    cv::Rect CropRegion;
    int CaptureFramerate;
    int FramerateDivider;
    std::string filter;
};

cv::Ptr<cv::aruco::Dictionary> GetArucoDict();

cv::Ptr<cv::aruco::DetectorParameters> GetArucoParams();

cv::Size GetScreenSize();

cv::Size GetFrameSize();

int GetCaptureFramerate();

CaptureConfig GetCaptureConfig();

CameraStartType GetCaptureMethod();

//list of downscales to be done to the aruco detections
std::vector<float> GetReductionFactor();

//list of resolutions in the end
std::vector<cv::Size> GetArucoReductions();

cv::UMat& GetArucoImage(int id);

struct WebsocketConfig
{
    bool Unix;
    bool TCP;
    bool Server;
    cv::String IP;
    int Port; 
    std::vector<std::string> URLs;
};

WebsocketConfig GetWebsocketConfig();

struct InternalCameraConfig
{
    std::string CameraName;
    cv::Affine3d LocationRelative;
};

std::vector<InternalCameraConfig> GetInternalCameraPositionsConfig();