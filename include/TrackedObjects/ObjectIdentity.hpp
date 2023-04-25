#pragma once

#include <cstdint>
#include <string>
#include <opencv2/core/affine.hpp>


enum class PacketType : uint8_t
{
	Null                = 0, //Invalid data
	Camera				= 0b1, //Object is camera
	ReferenceAbsolute   = 0b10, //The table or the basket, something that doesn't move when we're in external view
	ReferenceRelative   = 0b11, //Something that doesn't move and should have a fixed location, 
		//but we're moving (internal view), so that thing is moving relative to us, so here's the location of that thing relative to us
	Robot               = 0b100, //A robot. Ennemy or ally.
	TrackerCube			= 0b101, //Tracker that we put on the ennemy robot
	TopTracker			= 0b110, //Tracker that's on every robot
	Puck                = 0b1000, //A cake or something. 
	Tag					= 0b10000, // Raw tag. Size in metadata, ID is numeral. For debug/inspect purposes
};

struct __attribute__ ((packed)) PackedIdentity
{
	PacketType type;
	uint8_t numeral;
	uint16_t MetadataLength;
};

struct ObjectIdentity
{
	PacketType type;
	uint8_t numeral;
	std::string metadata;

	ObjectIdentity()
		:type(PacketType::Null), numeral(0), metadata("")
	{};

	ObjectIdentity(PacketType InType, uint8_t InNumeral)
		:type(InType), numeral(InNumeral), metadata("")
	{};

	size_t GetSize() const
	{
		return sizeof(PackedIdentity) + metadata.length(); //uint16 is for string length
	};

	operator PackedIdentity() const
	{
		return {type, numeral, (uint16_t)metadata.length()};
	};

	int PackInto(char* buffer, int maxlength) const;

	template<class T>
	T& GetTypeFromMetadata(size_t pos)
	{
		return *(T*)&metadata[pos];
	};

	template<class T>
	void AddTypeToMetadata(T value)
	{
		std::size_t pos = metadata.size();
		metadata.resize(pos + sizeof(T));
		GetTypeFromMetadata<T>(pos) = value;
	};
};

struct ObjectData
{
	ObjectIdentity identity;
	cv::Affine3d location;

	ObjectData(ObjectIdentity InIdentity = ObjectIdentity(), cv::Affine3d InLocation = cv::Affine3d::Identity())
		:identity(InIdentity), location(InLocation)
	{}

	ObjectData(ObjectIdentity InIdentity, double posX, double posY, double posZ)
		:identity(InIdentity)
	{
		location = cv::Affine3d::Identity();
		location.translation(cv::Vec3d(posX, posY, posZ));
	}
};
