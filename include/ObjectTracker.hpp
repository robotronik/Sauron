#pragma once

#include <opencv2/core.hpp>
#include "TrackedObjects/TrackedObject.hpp"


class ObjectTracker
{
private:
	vector<TrackedObject*> objects;
	int* ArucoMap;
	float* ArucoSizes;

public:
	ObjectTracker(/* args */);
	~ObjectTracker();

	void RegisterTrackedObject(TrackedObject* object);

	void SolveLocations(vector<Affine3d>& Cameras, vector<CameraView>& Tags);

	void DisplayObjects(viz::Viz3d* visualizer);

	//only needed for the center
	void SetArucoSize(int number, float SideLength);

	float GetArucoSize(int number);

private:

	void RegisterArucoRecursive(TrackedObject* object, int index);
};