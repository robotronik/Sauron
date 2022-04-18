#pragma once

#include <vector>
#include <time.h>
#include <stdint.h>
#include <string>
#include <cstring>

using namespace std;

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

	string ToCSV();
};

struct SerialTransmission
{
public:
	uint8_t NumPositions;
	uint16_t score; //score visuel, inclue uniquement les palets
	uint32_t ms; //enough for 49 days
	vector<PositionPacket> PositionPackets;

	const static uint64_t BaudRate;
	const static string StartFlag, StopFlag;
	
private:
	char* buffer;

public:
	SerialTransmission();
	~SerialTransmission();

	uint32_t GetSizeNoVector() const;
	uint32_t GetPacketSize() const;
	void* ToBuffer();
	bool FromBuffer(void* buffer, size_t length);
};
