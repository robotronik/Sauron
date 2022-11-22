#include "TrackedObjects/TrackerCube.hpp"

#include "TrackedObjects/ObjectIdentity.hpp"

#include "math3d.hpp"

using namespace cv;
using namespace std;

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

vector<ObjectData> TrackerCube::ToObjectData(int BaseNumeral)
{
	ObjectData robot;
	robot.identity.numeral = BaseNumeral;
	robot.identity.type = PacketType::Robot;
	robot.location = Location;
	return {robot};
}