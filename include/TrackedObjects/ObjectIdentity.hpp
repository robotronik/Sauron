#pragma once

#include <cstdint>
#include <opencv2/core/affine.hpp>


enum class PacketType : uint8_t
{
	Null                = 0, //Invalid data
	Camera				= 0b1, //Object is camera
	ReferenceAbsolute   = 0b10, //The table or the basket, something that doesn't move when we're in external view
	ReferenceRelative   = 0b100, //Something that doesn't move and should have a fixed location, but we're moving (internal view), so that thing is moving relative to us, so here's the location of that thing relative to us
	Robot               = 0b1000, //A robot. Ennemy or ally.
	Puck                = 0b10000, //A cake or something. 
	Tag					= 0b100000, // Raw tag. Size in metadata, ID is numeral. For debug/inspect purposes
};

struct __attribute__ ((packed)) ObjectIdentity
{
	PacketType type;
	uint8_t numeral;
	uint16_t metadata;

	ObjectIdentity()
		:type(PacketType::Null), numeral(0), metadata(0)
	{}

	ObjectIdentity(PacketType InType, uint8_t InNumeral)
		:type(InType), numeral(InNumeral), metadata(0)
	{}

};

struct ObjectData
{
	ObjectIdentity identity;
	cv::Affine3d location;
};
