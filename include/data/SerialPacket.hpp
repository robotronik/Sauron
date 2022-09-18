#pragma once

#include <vector>
#include <time.h>
#include <stdint.h>
#include <string>
#include <cstring>
#include "data/DataPacket.hpp"


struct SerialTransmission
{
public:
	uint8_t NumPositions;
	uint16_t score; //score visuel, inclue uniquement les palets
	uint32_t ms; //enough for 49 days
	std::vector<PositionPacket> PositionPackets;

	const static uint64_t BaudRate;
	const static std::string StartFlag, StopFlag;
	
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
