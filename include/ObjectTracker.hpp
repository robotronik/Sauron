#pragma once

#include <opencv2/core.hpp>
#include "TrackedObject.hpp"

struct CameraView
{
	int Camera;
	int TagID;
	Affine3d TagTransform;

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



class ObjectTracker
{
private:
	vector<TrackedObject*> objects;
	int* ArucoMap;
public:
	ObjectTracker(/* args */);
	~ObjectTracker();

	void RegisterTrackedObject(TrackedObject* object);

	void SolveLocations(vector<Affine3d> Cameras, vector<CameraView> Tags);

	void DisplayObjects(viz::Viz3d* visualizer);

private:

	void RegisterArucoRecursive(TrackedObject* object, int index);
};