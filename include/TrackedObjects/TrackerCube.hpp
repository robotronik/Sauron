#pragma once

#include "TrackedObjects/TrackedObject.hpp"

class TrackerCube : public TrackedObject
{
private:

public:
	TrackerCube(vector<int> MarkerIdx, float MarkerSize, cv::Point3d CubeSize, cv::String InName);
	~TrackerCube();

	virtual vector<ObjectData> ToObjectData(int BaseNumeral) override;
};