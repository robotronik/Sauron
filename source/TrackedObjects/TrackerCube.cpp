#include "TrackedObjects/TrackerCube.hpp"

#include "TrackedObjects/ObjectIdentity.hpp"

#include "math3d.hpp"

using namespace cv;
using namespace std;

TrackerCube::TrackerCube(vector<int> MarkerIdx, double MarkerSize, double Diameter, String InName)
{
	Unique = false;
	Name = InName;
	int numsides = MarkerIdx.size();
	vector<Point3d> Locations;
	Locations.resize(numsides);
	for (int i = 0; i < numsides; i++)
	{
		double angle = M_PI * i * 2.0 / numsides;
		double sinv, cosv;
		sincos(angle, &sinv, &cosv);
		Locations[i] = Point3d(cosv * Diameter/2, sinv * Diameter/2, 0);
	}
	
	//vector<Point3d> Locations = {Point3d(CubeSize.x/2.0,0,0), Point3d(0,CubeSize.y/2.0,0), Point3d(-CubeSize.x/2.0,0,0), Point3d(0,-CubeSize.y/2.0,0)};
	for (int i = 0; i < numsides; i++)
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
	robot.identity.metadata = markers[0].number;
	robot.location = Location;
	return {robot};
}