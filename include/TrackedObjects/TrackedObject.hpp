#pragma once

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <opencv2/core.hpp>     // Basic OpenCV structures (Mat, Scalar)
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  // OpenCV window I/O
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/viz.hpp>
#include "data/CameraView.hpp"

class Camera;
struct RobotPacket;

using namespace std;
using namespace cv;

struct ArucoMarker
{
	float sideLength;
	int number;
	Affine3d Pose;

	static vector<Point3d> GetObjectPointsNoOffset(float SideLength)
	{
		float sql2 = SideLength*0.5;
		return {
			Point3d(-sql2, sql2, 0.0),
			Point3d(sql2, sql2, 0.0),
			Point3d(sql2, -sql2, 0.0),
			Point3d(-sql2, -sql2, 0.0)
		};
	}

	ArucoMarker()
		:sideLength(0.05),
		number(-1),
		Pose(Affine3d::Identity())
	{}

	ArucoMarker(float InSideLength, int InNumber)
		:sideLength(InSideLength),
		number(InNumber),
		Pose(Affine3d::Identity())
	{}

	ArucoMarker(float InSideLength, int InNumber, Affine3d InPose)
		:sideLength(InSideLength),
		number(InNumber),
		Pose(InPose)
	{}

	void DisplayMarker(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName);
};

class TrackedObject
{
public:
	vector<ArucoMarker> markers;
	vector<TrackedObject*> childs;
	Affine3d Location;
	bool Unique;
	String Name;

public:

	TrackedObject()
	{};

	virtual Affine3d ResolveLocation(vector<Affine3d>& Cameras, vector<CameraView>& views);

	virtual void DisplayRecursive(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName);
};

class TrackerCube : public TrackedObject
{
private:

public:
	TrackerCube(vector<int> MarkerIdx, float MarkerSize, Point3d CubeSize, String InName);
	~TrackerCube();

	RobotPacket ToPacket(int RobotId);

	virtual void DisplayRecursive(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName) override;
};



extern ArucoMarker center;

vector<Point2f> ReorderMarkerCorners(vector<Point2f> Corners);

Affine3d GetTagTransform(float SideLength, std::vector<Point2f> Corners, Camera* Cam);

Affine3d GetTransformRelativeToTag(ArucoMarker& Tag, std::vector<Point2f> Corners, Camera* Cam);

void Tracker3DTest();