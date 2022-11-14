#pragma once

#include "TrackedObjects/TrackedObject.hpp"

class StaticObject : public TrackedObject
{
private:
	bool Relative; //Allow modifying the position of this object ?
public:
	StaticObject(bool InRelative, cv::String InName);
	~StaticObject();

	virtual bool SetLocation(cv::Affine3d InLocation) override;

	virtual vector<ObjectData> ToObjectData(int BaseNumeral) override;

	virtual void DisplayRecursive2D(BoardViz2D* visualizer, cv::Affine3d RootLocation, cv::String rootName) override;

	virtual void DisplayRecursive(cv::viz::Viz3d* visualizer, cv::Affine3d RootLocation, cv::String rootName) override;
};