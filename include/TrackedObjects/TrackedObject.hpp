#pragma once

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#ifdef WITH_VTK
#include <opencv2/viz.hpp>
#endif
#include "visualisation/BoardViz2D.hpp"
#include "TrackedObjects/ObjectIdentity.hpp"

class ArucoCamera;
class BoardViz2D;
struct CameraView;
struct CameraArucoData;
struct ArucoMarker
{
	float sideLength;
	int number;
	cv::Affine3d Pose;

	std::vector<cv::Point3d> ObjectPointsNoOffset;

	static std::vector<cv::Point3d> GetObjectPointsNoOffset(float SideLength);

	std::vector<cv::Point3d>& GetObjectPointsNoOffset();

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
	#ifdef WITH_VTK
	void DisplayMarker(cv::viz::Viz3d* visualizer, cv::Affine3d RootLocation, cv::String rootName);
	#endif
};

class TrackedObject
{
public: 
	struct ArucoViewCameraLocal
	{
		cv::Affine3d AccumulatedTransform; //transform to marker, not including the marker's transform relative to it's parent
		ArucoMarker* Marker; //pointer to source marker
		std::vector<cv::Point2f> CameraCornerPositions; //corner positions, in space relative to the calling object's coordinates
		std::vector<cv::Point3d> LocalMarkerCorners; //corner positions in camera image space
		int IndexInCameraData; //index where this marker was found in the camera
	};
public:
	std::vector<ArucoMarker> markers;
	std::vector<TrackedObject*> childs;
	bool Unique;
	bool CoplanarTags;
	cv::String Name;

protected:
	cv::Affine3d Location;

public:

	TrackedObject()
		:Location(cv::Affine3d::Identity()),
		Unique(true),
		CoplanarTags(false)
	{};

	virtual bool SetLocation(cv::Affine3d InLocation);
	virtual cv::Affine3d GetLocation();

	virtual cv::Affine3d ResolveLocation(std::vector<cv::Affine3d>& Cameras, std::vector<CameraView>& views);

	//Find the parameters and the accumulated transform of the tag in the component and it's childs
	virtual bool FindTag(int MarkerID, ArucoMarker& Marker, cv::Affine3d& TransformToMarker);

	//Returns all the corners in 3D space of this object and it's childs, with the marker ID. Does not clear the array at start.
	virtual void GetObjectPoints(std::vector<std::vector<cv::Point3d>>& MarkerCorners, std::vector<int>& MarkerIDs, cv::Affine3d rootTransform = cv::Affine3d::Identity(), std::vector<int> filter = {});

	//Returns the surface area, markers that are seen by the camera that belong to this object or it's childs are stored in MarkersSeen
	virtual float GetSeenMarkers(const CameraArucoData& CameraData, std::vector<ArucoViewCameraLocal> &MarkersSeen, cv::Affine3d AccumulatedTransform = cv::Affine3d::Identity());

	float ReprojectSeenMarkers(const std::vector<ArucoViewCameraLocal> &MarkersSeen, const cv::Mat &rvec, const cv::Mat &tvec, const CameraArucoData &CameraData);
	//Given corners, solve this object's location using multiple tags at once
	//Output transform is given relative to the camera
	virtual cv::Affine3d GetObjectTransform(const CameraArucoData& CameraData, float& Surface, float& ReprojectionError);

	virtual std::vector<ObjectData> ToObjectData(int BaseNumeral);
};



cv::Affine3d GetTagTransform(float SideLength, std::vector<cv::Point2f> Corners, cv::Mat& CameraMatrix, cv::Mat& DistanceCoefficients);

cv::Affine3d GetTagTransform(float SideLength, std::vector<cv::Point2f> Corners, ArucoCamera* Cam);

cv::Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<cv::Point2f> Corners, ArucoCamera* Cam);