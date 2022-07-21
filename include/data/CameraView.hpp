#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

using namespace cv;
using namespace std;

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

struct CameraArucoData
{
	Affine3d CameraTransform;
	Mat CameraMatrix;
	Mat DistanceCoefficients;
	vector<int> TagIDs;
	vector<vector<Point2f>> TagCorners;

};
