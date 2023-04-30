#include "TrackedObjects/TopTracker.hpp"

#include "TrackedObjects/ObjectIdentity.hpp"

#include "math3d.hpp"
#include "metadata.hpp"

using namespace cv;
using namespace std;

TopTracker::TopTracker(int MarkerIdx, double MarkerSize, String InName)
{
	Unique = false;
	Name = InName;
	markers.clear();
	Affine3d markertransform = Affine3d::Identity();
	ArucoMarker marker(MarkerSize, MarkerIdx, markertransform);
	markers.push_back(marker);
}

TopTracker::~TopTracker()
{
}

vector<ObjectData> TopTracker::ToObjectData(int BaseNumeral)
{
	ObjectData robot;
	robot.identity.numeral = BaseNumeral;
	robot.identity.type = PacketType::TopTracker;
	for (int i = 0; i < markers.size(); i++)
	{
		AddTypeToMetadata<uint8_t>(robot.identity.metadata, markers[i].number);
	}
	robot.location = Location;
	return {robot};
}