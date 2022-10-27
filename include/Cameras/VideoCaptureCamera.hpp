#pragma once

#include <iostream>
#include <string>   // for strings
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>

#include <opencv2/cudacodec.hpp>

#include "Cameras/Camera.hpp"
#include "data/ImageTypes.hpp"


class VideoCaptureCamera : public ArucoCamera
{


private:
	//capture using classic api
	cv::VideoCapture* feed;

public:

	VideoCaptureCamera(CameraSettings InSettings)
		:ArucoCamera(InSettings)
	{
	}

	~VideoCaptureCamera()
	{
		if (connected)
		{
			delete feed;
		}
	}

	//Start the camera
	virtual bool StartFeed() override;

	//Lock a frame to be capture at this time
	//This allow for simultaneous capture
	virtual bool Grab(int BufferIndex) override;

	//Retrieve or read a frame
	virtual bool Read(int BufferIndex) override;

	virtual bool InjectImage(int BufferIndex, cv::UMat& frame) override;

};