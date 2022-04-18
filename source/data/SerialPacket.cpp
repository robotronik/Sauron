#include "data/SerialPacket.hpp"
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <math.h>

string PositionPacket::ToCSV()
{
	ostringstream fab; 
	fab << (int)type << ", " << (int)numeral
	<< ", " << X << ", " << Y << ", " << rotation*180/M_PI;
	return fab.str();
}

const uint64_t SerialTransmission::BaudRate = 115200;
const string SerialTransmission::StartFlag = string("robotronik");
const string SerialTransmission::StopFlag = string("meilleurclub");

SerialTransmission::SerialTransmission()
{
	NumPositions = 0;
	score = 0;
	ms = 0;
	buffer = NULL;
}

SerialTransmission::~SerialTransmission()
{
	if (buffer != NULL)
	{
		delete buffer;
	}
}

uint32_t SerialTransmission::GetSizeNoVector() const
{
	return sizeof(SerialTransmission) - sizeof(PositionPackets) - sizeof(buffer);
}

uint32_t SerialTransmission::GetPacketSize() const
{
	return GetSizeNoVector()
	+ NumPositions * sizeof(PositionPacket);
}

void* SerialTransmission::ToBuffer()
{
	uint32_t buffsize = GetPacketSize();
	if (buffer != NULL)
	{
		delete buffer;
	}
	buffer = new char[buffsize];
	char* copyptr = buffer;
	memcpy(copyptr, this, GetSizeNoVector());
	copyptr += GetSizeNoVector();
	memcpy(copyptr, &PositionPackets[0], NumPositions * sizeof(PositionPacket));
	/*for (size_t i = 0; i < NumPositions; i++)
	{
		cout << PositionPackets[i].ToCSV() << endl;
	}
	for (size_t i = 0; i < buffsize; i++)
	{
		printf("%02hhx ", buffer[i]);
	}
	printf("\r\n");*/
	return reinterpret_cast<void*>(buffer);
}

bool SerialTransmission::FromBuffer(void* buffer, size_t length)
{
	try
	{
		size_t length_so_far = GetSizeNoVector();
		if (length < length_so_far)
		{
			return false;
		}
		
		char* buffptr = reinterpret_cast<char*>(buffer);
		memcpy(this, buffptr, GetSizeNoVector());
		buffptr += GetSizeNoVector();
		length_so_far += NumPositions * sizeof(PositionPacket);
		if (length < length_so_far)
		{
			return false;
		}
		PositionPackets.resize(NumPositions);
		memcpy(&PositionPackets[0], buffptr, NumPositions * sizeof(PositionPacket));
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
		return false;
	}
	return true;
}