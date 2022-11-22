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

	void UnregisterTrackedObject(TrackedObject* object);

	void SolveLocationsPerObject(vector<CameraArucoData>& CameraData);

	void SolveLocationsTagByTag(vector<cv::Affine3d>& Cameras, vector<CameraView>& Tags);

	std::vector<ObjectData> GetObjectDataVector();

	//only needed for the center
	void SetArucoSize(int number, float SideLength);

	float GetArucoSize(int number);

private:

	void RegisterArucoRecursive(TrackedObject* object, int index);
};