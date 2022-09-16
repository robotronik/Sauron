#include "TrackedObjects/TrackerCube.hpp"

#include "math3d.hpp"
#include "data/DataPacket.hpp"

TrackerCube::TrackerCube(vector<int> MarkerIdx, float MarkerSize, Point3d CubeSize, String InName)
{
	Unique = false;
	Name = InName;
	vector<Point3d> Locations = {Point3d(CubeSize.x/2.0,0,0), Point3d(0,CubeSize.y/2.0,0), Point3d(-CubeSize.x/2.0,0,0), Point3d(0,-CubeSize.y/2.0,0)};
	for (int i = 0; i < 4; i++)
	{
		Matx33d markerrot = MakeRotationFromZY(Locations[i], Vec3d(0,0,1));
		Affine3d markertransform = Affine3d(markerrot, Locations[i]);
		ArucoMarker marker(MarkerSize, MarkerIdx[i], markertransform);
		if (marker.number >= 0)
		{
			markers.push_back(marker);
		}
	}
	
}

TrackerCube::~TrackerCube()
{
}

vector<PositionPacket> TrackerCube::ToPacket(int BaseNumeral)
{
	PositionPacket robot;
	robot.type = PacketType::Robot;
	robot.X = Location.translation()[0];
	robot.Y = Location.translation()[1];
	robot.rotation = GetRotZ(Location.rotation());
	robot.numeral = BaseNumeral;
	return {robot};
}

void TrackerCube::DisplayRecursive(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName)
{
	viz::WText3D Robotext(Name, (RootLocation*Location).translation(), 0.01);
	visualizer->showWidget(Name, Robotext);
	TrackedObject::DisplayRecursive(visualizer, RootLocation, rootName);
}