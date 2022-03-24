#include "data/SerialPacket.hpp"
#include <memory.h>
#include <stdio.h>
#include <iostream>

SerialPacketOut::SerialPacketOut()
{
	NumRobots = 0;
	NumPalets = 0;
	score = 0;
	ms = 0;
	buffer = NULL;
}

SerialPacketOut::~SerialPacketOut()
{
	if (buffer)
	{
		delete buffer;
	}
}

uint32_t SerialPacketOut::GetSizeNoVector() const
{
	return sizeof(SerialPacketOut) - sizeof(robots) - sizeof(palets) - sizeof(buffer);
}

uint32_t SerialPacketOut::GetPacketSize() const
{
	return GetSizeNoVector()
	+ NumRobots * sizeof(RobotPacket) + NumPalets * sizeof(PaletPacket);
}

void* SerialPacketOut::ToBuffer()
{
	uint32_t buffsize = GetPacketSize();
	if (buffer)
	{
		delete buffer;
	}
	buffer = new char[buffsize];
	char* copyptr = buffer;
	memcpy(copyptr, this, GetSizeNoVector());
	copyptr += GetSizeNoVector();
	memcpy(copyptr, robots.data(), NumRobots * sizeof(RobotPacket));
	copyptr += NumRobots * sizeof(RobotPacket);
	memcpy(copyptr, palets.data(), NumPalets * sizeof(PaletPacket));
	return reinterpret_cast<void*>(buffer);
}

bool SerialPacketOut::FromBuffer(void* buffer)
{
	try
	{
		char* buffptr = reinterpret_cast<char*>(buffer);
		memcpy(this, buffptr, GetSizeNoVector());
		buffptr += GetSizeNoVector();
		robots.resize(NumRobots);
		palets.resize(NumPalets);
		memcpy(robots.data(), buffptr, NumRobots * sizeof(RobotPacket));
		buffptr += NumRobots * sizeof(RobotPacket);
		memcpy(palets.data(), buffptr, NumPalets * sizeof(PaletPacket));
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		return false;
	}
	return true;
}