#include "TrackedObjects/StaticObject.hpp"

#include "math3d.hpp"
#include "data/DataPacket.hpp"

using namespace cv;
using namespace std;

StaticObject::StaticObject(bool InRelative, String InName)
{
	Unique = true;
	Relative = InRelative;
	Name = InName;
    double yamp = 1-0.570, xamp = 1.5-0.575;
    double size = 0.1;
    vector<int> numbers = {20, 22, 21, 23};
	for (int i = 0; i < 4; i++)
	{
		Matx33d markerrot = MakeRotationFromZX(Vec3d(0,0,1), Vec3d(0,1,0));
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
	if (Relative)
	{
		Location = InLocation;
		return true;
	}
	
	Location = Affine3d::Identity();
	return false;
}

vector<PositionPacket> StaticObject::ToPacket(int BaseNumeral)
{
	PositionPacket packet;
	packet.type = PacketType::Reference;
	Affine3dToVictor(packet, Location);
	packet.numeral = BaseNumeral;
	return {packet};
}

void StaticObject::DisplayRecursive(viz::Viz3d* visualizer, Affine3d RootLocation, String rootName)
{
	viz::WText3D Robotext(Name, (RootLocation*Location).translation(), 0.01);
	visualizer->showWidget(Name, Robotext);
	TrackedObject::DisplayRecursive(visualizer, RootLocation, rootName);
}