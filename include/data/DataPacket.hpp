#pragma once

#include <vector>
#include <time.h>
#include <stdint.h>
#include <string>


enum class PacketType : uint8_t
{
	Null = 0,
	Robot = 1,
	Puck = 3,
	Reference = 7
};

struct PositionPacket //palets sur l'arène uniquement
{
	PacketType type;
	uint8_t numeral;
	float X;
	float Y;
	float rotation; //-pi to pi

	PositionPacket()
	:type(PacketType::Null),
	numeral(UINT8_MAX), X(0), Y(0), rotation(0)
	{};

	PositionPacket(PacketType InType, uint8_t InNumber, float InX, float InY, float InRotation)
	:type(InType), numeral(InNumber), X(InX), Y(InY), rotation(InRotation)
	{};

	std::string ToCSV();
};