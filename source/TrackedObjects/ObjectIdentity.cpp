#include "TrackedObjects/ObjectIdentity.hpp"
#include <cassert>
#include <map>
#include "math3d.hpp"
#include "metadata.hpp"
#include "visualisation/BoardGL.hpp"
using namespace std;



int ObjectIdentity::PackInto(char* buffer, int maxlength) const
{
	int size = GetSize();
	assert(size <= maxlength);
	
	PackedIdentity ph(*this);
	memcpy(buffer, &ph, sizeof(ph));
	buffer += sizeof(ph);
	memcpy(buffer, metadata.data(), metadata.size());
	return size;
}

int ObjectIdentity::UnpackFrom(const char* buffer, int maxlength)
{
	assert(maxlength >= sizeof(PackedIdentity));
	const PackedIdentity* pi = reinterpret_cast<const PackedIdentity*>(buffer);
	numeral = pi->numeral;
	type = pi->type;
	assert(maxlength >= sizeof(PackedIdentity) + pi->MetadataLength);
	metadata = string(buffer + sizeof(PackedIdentity), pi->MetadataLength);
	return sizeof(PackedIdentity) + pi->MetadataLength;
}

std::optional<GLObject> ObjectData::ToGLObject() const
{
	static const map<enum PacketType, enum MeshNames> PacketToMesh = 
	{
		{PacketType::Robot, MeshNames::robot},
		{PacketType::Camera, MeshNames::brio},
		{PacketType::ReferenceAbsolute, MeshNames::arena},
		{PacketType::ReferenceRelative, MeshNames::arena},
		{PacketType::Tag, MeshNames::tag},
		{PacketType::TopTracker, MeshNames::toptracker},
		{PacketType::TrackerCube, MeshNames::trackercube},

		{PacketType::PinkCake, MeshNames::pinkcake},
		{PacketType::YellowCake, MeshNames::yellowcake},
		{PacketType::BrownCake, MeshNames::browncake},
		{PacketType::Cherry, MeshNames::cherry}
	};

	auto foundmesh = PacketToMesh.find(identity.type);
	if (foundmesh == PacketToMesh.end())
	{
		return nullopt;
	}

	GLObject obj;
	obj.type = foundmesh->second;
	obj.location = Affine3DToGLM(location);
	obj.metadata = identity.metadata;
	return obj;
}

vector<GLObject> ObjectData::ToGLObjects(const vector<ObjectData>& data)
{
	vector<GLObject> outobj;
	outobj.reserve(data.size());
	for (int i = 0; i < data.size(); i++)
	{
		auto obj = data[i].ToGLObject();
		if (obj.has_value())
		{
			outobj.push_back(obj.value());
		}
	}
	return outobj;
}