#pragma once

#include <opencv2/core.hpp>
#include "TrackedObjects/TrackedObject.hpp"

//Class that handles the objects, and holds information about each tag's size
//Registered objects will have their locations solved and turned into a vector of ObjectData for display and data sending
class ObjectTracker
{
private:
	std::vector<TrackedObject*> objects;
	int ArucoMap[100]; //Which object owns the tag at index i ? objects[ArucoMap[TagID]]
	float ArucoSizes[100]; //Size of the aruco tag

public:
	ObjectTracker(/* args */);
	~ObjectTracker();

	void RegisterTrackedObject(TrackedObject* object);

	void UnregisterTrackedObject(TrackedObject* object);

	void SolveLocationsPerObject(const std::vector<CameraArucoData>& CameraData, unsigned long tick);

	void SolveLocationsTagByTag(std::vector<cv::Affine3d>& Cameras, std::vector<CameraView>& Tags);

	std::vector<ObjectData> GetObjectDataVector();

	//only needed for the center
	void SetArucoSize(int number, float SideLength);

	float GetArucoSize(int number);

private:

	void RegisterArucoRecursive(TrackedObject* object, int index);
};