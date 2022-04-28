#pragma once

#include <iostream>
#include <string>   // for strings
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>

#include <opencv2/cudacodec.hpp>
#include <opencv2/cudawarping.hpp>

#include "thirdparty/list-devices.hpp"

#include "Camera.hpp"

using namespace std;
using namespace cv;

class FisheyeCamera : public Camera
{
private:
	bool HasUndistortionMaps;
	cuda::GpuMat UndistMap1, UndistMap2;

public:
	FisheyeCamera(CameraSettings InSettings)
	:Camera(InSettings)
	{}

	virtual void Undistort(int BufferIdx) override;

	//Create lower-resolution copies of the frame to be used in aruco detection
	virtual void RescaleFrames(int BufferIdx) override;

	virtual void Calibrate(vector<vector<Point3f>> objectPoints,
	vector<vector<Point2f>> imagePoints, Size imageSize,
	Mat& cameraMatrix, Mat& distCoeffs,
	OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs) override;
};