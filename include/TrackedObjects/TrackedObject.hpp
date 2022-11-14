#pragma once

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/viz.hpp>

#include "visualisation/BoardViz2D.hpp"
#include "TrackedObjects/ObjectIdentity.hpp"

class ArucoCamera;
class BoardViz2D;
struct CameraView;
struct CameraArucoData;

using std::vector;
struct ArucoMarker
{
	float sideLength;
	int number;
	cv::Affine3d Pose;

	vector<cv::Point3d> ObjectPointsNoOffset;

	static vector<cv::Point3d> GetObjectPointsNoOffset(float SideLength);

	vector<cv::Point3d>& GetObjectPointsNoOffset();

	ArucoMarker()
		:sideLength(0.05),
		number(-1),
		Pose(cv::Affine3d::Identity())
	{}

	ArucoMarker(float InSideLength, int InNumber)
		:sideLength(InSideLength),
		number(InNumber),
		Pose(cv::Affine3d::Identity()),
		ObjectPointsNoOffset(GetObjectPointsNoOffset(InSideLength))
	{}

	ArucoMarker(float InSideLength, int InNumber, cv::Affine3d InPose)
		:sideLength(InSideLength),
		number(InNumber),
		Pose(InPose),
		ObjectPointsNoOffset(GetObjectPointsNoOffset(InSideLength))
	{}

	void DisplayMarker(cv::viz::Viz3d* visualizer, cv::Affine3d RootLocation, cv::String rootName);
};

class TrackedObject
{
public:
	vector<ArucoMarker> markers;
	vector<TrackedObject*> childs;
	bool Unique;
	cv::String Name;

protected:
	cv::Affine3d Location;

public:

	TrackedObject()
	{};

	virtual bool SetLocation(cv::Affine3d InLocation);
	virtual cv::Affine3d GetLocation();

	virtual cv::Affine3d ResolveLocation(vector<cv::Affine3d>& Cameras, vector<CameraView>& views);

	//Find the parameters and the accumulated transform of the tag in the component and it's childs
	virtual bool FindTag(int MarkerID, ArucoMarker& Marker, cv::Affine3d& TransformToMarker);

	//Returns all the corners in 3D space of this object and it's childs, with the marker ID. Does not clear the array at start.
	virtual void GetObjectPoints(vector<vector<cv::Point3d>>& MarkerCorners, vector<int>& MarkerIDs, cv::Affine3d rootTransform = cv::Affine3d::Identity(), vector<int> filter = {});

	//Given corners, solve this object's location using multiple tags at once
	virtual cv::Affine3d GetObjectTransform(CameraArucoData& CameraData, float& Surface);

	virtual void DisplayRecursive2D(BoardViz2D* visualizer, cv::Affine3d RootLocation, cv::String rootName);

	virtual void DisplayRecursive(cv::viz::Viz3d* visualizer, cv::Affine3d RootLocation, cv::String rootName);

	virtual vector<ObjectData> ToObjectData(int BaseNumeral);
};



cv::Affine3d GetTagTransform(float SideLength, std::vector<cv::Point2f> Corners, cv::Mat& CameraMatrix, cv::Mat& DistanceCoefficients);

cv::Affine3d GetTagTransform(float SideLength, std::vector<cv::Point2f> Corners, ArucoCamera* Cam);

cv::Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<cv::Point2f> Corners, ArucoCamera* Cam);

void Tracker3DTest();