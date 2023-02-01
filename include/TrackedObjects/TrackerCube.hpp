#pragma once

#include "TrackedObjects/TrackedObject.hpp"

//A cube with 4 tags, one on each side
class TrackerCube : public TrackedObject
{
private:

public:
	TrackerCube(std::vector<int> MarkerIdx, float MarkerSize, cv::Point3d CubeSize, cv::String InName);
	~TrackerCube();

	virtual std::vector<ObjectData> ToObjectData(int BaseNumeral) override;
};