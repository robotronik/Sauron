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

	void SolveLocationsPerObject(vector<CameraArucoData>& CameraData);

	void SolveLocationsTagByTag(vector<cv::Affine3d>& Cameras, vector<CameraView>& Tags);

	void DisplayObjects2D(BoardViz2D* visualizer);

	void DisplayObjects(cv::viz::Viz3d* visualizer);

	//only needed for the center
	void SetArucoSize(int number, float SideLength);

	float GetArucoSize(int number);

private:

	void RegisterArucoRecursive(TrackedObject* object, int index);
};