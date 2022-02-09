#pragma once

#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>

using namespace std;
using namespace cv;

Ptr<aruco::Dictionary> GetArucoDict();

Ptr<aruco::DetectorParameters> GetArucoParams();

UMat& GetArucoImage(int id);