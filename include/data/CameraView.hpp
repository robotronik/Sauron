#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

using namespace cv;

struct CameraView
{
	int Camera;
	int TagID;
	Affine3d TagTransform;
	double score;

	CameraView()
		:Camera(0),
		TagID(0),
		TagTransform(Affine3d::Identity())
	{}

	CameraView(int InCamera, int InTagID, Affine3d InTagTransform)
		:Camera(InCamera),
		TagID(InTagID),
		TagTransform(InTagTransform)
	{}
};