#pragma once

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

Ptr<aruco::Dictionary> GetArucoDict();

Ptr<aruco::DetectorParameters> GetArucoParams();

Size GetScreenSize();

Size GetFrameSize();

int GetCaptureFramerate();

//list of downscales to be done to the aruco detections
vector<float> GetReductionFactor();

//list of resolutions in the end
vector<Size> GetArucoReductions();

UMat& GetArucoImage(int id);