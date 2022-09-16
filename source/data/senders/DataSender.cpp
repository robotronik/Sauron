#include "data/senders/DataSender.hpp"

#include <cinttypes>

#include <iostream>
#include <string>

void DataSender::PrintCSVHeader(ofstream &file)
{
	file << "ms" << ", ";
	for (size_t i = 0; i < SerialObjects.size(); i++)
	{
		file << "numeral, X , Y, rot(deg)";
		if (i != SerialObjects.size() -1)
		{
			file << ", ";
		}
		
	}
	file << endl;
}

void DataSender::PrintCSV(ofstream &file)
{
	uint32_t ms = (getTickCount() - StartTick) *1000 / getTickFrequency();
	file << ms << ", ";
	for (size_t i = 0; i < SerialObjects.size(); i++)
	{
		vector<PositionPacket> packets = SerialObjects[i]->ToPacket(i);
		for (size_t j = 0; j < packets.size(); j++)
		{
			file << packets[j].ToCSV();
			if ((i != SerialObjects.size() -1) || (j != packets.size() -1))
			{
				file << ", ";
			}
		}
	}
	file << endl;
}