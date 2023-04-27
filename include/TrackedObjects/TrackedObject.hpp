#pragma once

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <sstream>  // string to number conversion
#include <opencv2/core.hpp>		// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/video/tracking.hpp>
#include "TrackedObjects/ObjectIdentity.hpp"

class ArucoCamera;
class BoardViz2D;
struct CameraView;
struct CameraArucoData;
struct ArucoMarker
{
	float sideLength; //Length of a side of the tags. Tag should be square
	int number; //Number of the tag on it
	cv::Affine3d Pose; //Location relative to it's parent

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
};

//Base class for any object that should be tracked in 3D space.
//Can hold other object and/or Aruco tags
//Aruco tags will be registered to this object when added to the object tracker
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
	std::vector<ArucoMarker> markers; //Should be populated before adding to the Object Tracker
	std::vector<TrackedObject*> childs; //Should be populated before adding to the Object Tracker
	bool Unique; //Can there be only one ?
	bool CoplanarTags; //Are all tags on the same plane ? If true, then it uses IPPE solve when multiple tags are located
	cv::String Name; //Display name

protected:
	cv::Affine3d Location;
	unsigned long LastSeenTick = 0;
	cv::KalmanFilter LocationFilter;

public:

	TrackedObject();

	virtual bool SetLocation(cv::Affine3d InLocation, unsigned long tick);
	unsigned long GetLastSeenTick() { return LastSeenTick; }

	virtual bool ShouldBeDisplayed(unsigned long Tick);
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

	void Inspect();
};



cv::Affine3d GetTagTransform(float SideLength, std::vector<cv::Point2f> Corners, cv::Mat& CameraMatrix, cv::Mat& DistanceCoefficients);

cv::Affine3d GetTagTransform(float SideLength, std::vector<cv::Point2f> Corners, ArucoCamera* Cam);

cv::Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<cv::Point2f> Corners, ArucoCamera* Cam);