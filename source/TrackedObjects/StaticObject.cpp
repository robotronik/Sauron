#include "TrackedObjects/StaticObject.hpp"

#include "math3d.hpp"
#include "data/SerialPacket.hpp"

StaticObject::StaticObject(String InName)
{
	Unique = true;
	Name = InName;
    double xamp = 0.43, yamp = 0.925;
    double size = 0.14;
    vector<int> numbers = {20, 21, 22, 23};
	for (int i = 0; i < 4; i++)
	{
		Matx33d markerrot = MakeRotationFromXY(Vec3d(0,1,0), Vec3d(-1,0,0));
        Vec3d pos = Vec3d(i%2 ? xamp : -xamp, i>=2 ? yamp : -yamp, 0);
		Affine3d markertransform = Affine3d(markerrot, pos);
		ArucoMarker marker(size, numbers[i], markertransform);
		if (marker.number >= 0)
		{
			markers.push_back(marker);
		}
	}
	
}

StaticObject::~StaticObject()
{
}

bool StaticObject::SetLocation(Affine3d InLocation)
{
	Location = Affine3d::Identity();
	return false;
}

vector<PositionPacket> StaticObject::ToPacket(int BaseNumeral)
{
	PositionPacket robot;
	robot.type = PacketType::Robot;
	robot.X = Location.translation()[0];
	robot.Y = Location.translation()[1];
	robot.rotation = GetRotZ(Location.rotation());
	robot.numeral = BaseNumeral;
	return {robot};
}

void StaticObject::DisplayRecursive(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName)
{
	viz::WText3D Robotext(Name, (RootLocation*Location).translation(), 0.01);
	visualizer->showWidget(Name, Robotext);
	TrackedObject::DisplayRecursive(visualizer, RootLocation, rootName);
}