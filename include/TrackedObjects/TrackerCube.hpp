#pragma once

#include "TrackedObjects/TrackedObject.hpp"

class TrackerCube : public TrackedObject
{
private:

public:
	TrackerCube(vector<int> MarkerIdx, float MarkerSize, Point3d CubeSize, String InName);
	~TrackerCube();

	RobotPacket ToPacket(int RobotId);

	virtual void DisplayRecursive(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName) override;
};