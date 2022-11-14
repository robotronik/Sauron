#pragma once

#include "TrackedObjects/TrackedObject.hpp"

class TrackerCube : public TrackedObject
{
private:

public:
	TrackerCube(vector<int> MarkerIdx, float MarkerSize, cv::Point3d CubeSize, cv::String InName);
	~TrackerCube();

	virtual vector<ObjectData> ToObjectData(int BaseNumeral) override;

	virtual void DisplayRecursive2D(BoardViz2D* visualizer, cv::Affine3d RootLocation, cv::String rootName) override;

	virtual void DisplayRecursive(cv::viz::Viz3d* visualizer, cv::Affine3d RootLocation, cv::String rootName) override;
};