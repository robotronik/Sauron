#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>

//Represents a single tag seen by a camera in 3D space
//Deprecated
//Score is based on area
struct CameraView
{
	int Camera;
	int TagID;
	cv::Affine3d TagTransform;
	double score;

	CameraView()
		:Camera(0),
		TagID(0),
		TagTransform(cv::Affine3d::Identity()),
		score(0)
	{}

	CameraView(int InCamera, int InTagID, cv::Affine3d InTagTransform)
		:Camera(InCamera),
		TagID(InTagID),
		TagTransform(InTagTransform)
	{}
};

//Represents all the aruco tags seen by a single camera, as well as it's data need to recontruct objects in 3D space
struct CameraArucoData
{
	cv::Affine3d CameraTransform;
	cv::Mat CameraMatrix;
	cv::Mat DistanceCoefficients;
	std::vector<int> TagIDs;
	std::vector<std::vector<cv::Point2f>> TagCorners;
	class ArucoCamera *SourceCamera;

};
