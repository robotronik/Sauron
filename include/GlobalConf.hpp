#pragma once

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>

//Defines all global config parameters, and also reads the config file.

enum class CameraStartType;

struct CaptureConfig
{
	int StartType; //See CameraStartType in data/ImageTypes.hpp . Chooses method to use to start the camera
	cv::Size FrameSize; //resolution after crop
	cv::Rect CropRegion; //how many pixels to remove from each side ? only with nvvidconv/jetson/gstreamer
	float ReductionFactor; //factor to downscale image before aruco detection
	int CaptureFramerate;
	int FramerateDivider;
	std::string filter; //filter to block or allow certain cameras. If camera name contains the filter string, it's allowed. If the filter string starts with a !, the filter is inverted
};

cv::Ptr<cv::aruco::Dictionary> GetArucoDict();

cv::Ptr<cv::aruco::DetectorParameters> GetArucoParams();

cv::Size GetScreenSize();

cv::Size GetFrameSize();

int GetCaptureFramerate();

CaptureConfig GetCaptureConfig();

CameraStartType GetCaptureMethod();

//list of downscales to be done to the aruco detections
float GetReductionFactor();

//list of resolutions in the end
cv::Size GetArucoReduction();

cv::UMat& GetArucoImage(int id);

struct WebsocketConfig
{
	std::string Interface;
	bool TCP;
	bool Server;
	std::string IP;
	int Port; 
};

WebsocketConfig& GetWebsocketConfig();

struct InternalCameraConfig
{
	std::string CameraName;
	cv::Affine3d LocationRelative;
};

std::vector<InternalCameraConfig>& GetInternalCameraPositionsConfig();