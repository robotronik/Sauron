#include "data/SerialPacket.hpp"
#include <memory.h>

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