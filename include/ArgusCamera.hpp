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

class ArgusCamera : public Camera
{

public:
	ArgusCamera(CameraSettings InSettings)
	:Camera(InSettings)
	{}

	virtual bool Grab(int BufferIndex) override;
	
	virtual bool Read(int BufferIndex) override;
};

int ArgusEGLImage(int argc, char** argv);