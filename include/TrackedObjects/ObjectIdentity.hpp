#pragma once

#include <cstdint>
#include <opencv2/core/affine.hpp>

enum class PacketType : uint8_t
{
	Null                = 0,
	ReferenceAbsolute   = 0b1,
	ReferenceRelative   = 0b11,
	Robot               = 0b111,
	Puck                = 0b1111
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
