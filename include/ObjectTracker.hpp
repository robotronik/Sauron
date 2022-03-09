#pragma once

#include <opencv2/core.hpp>
#include "TrackedObject.hpp"


class ObjectTracker
{
private:
	vector<TrackedObject*> objects;
	int* ArucoMap;
public:
	ObjectTracker(/* args */);
	~ObjectTracker();

	void RegisterTrackedObject(TrackedObject* object);

	void SolveLocations(vector<Affine3d>& Cameras, vector<CameraView>& Tags);

	void DisplayObjects(viz::Viz3d* visualizer);

private:

	void RegisterArucoRecursive(TrackedObject* object, int index);
};