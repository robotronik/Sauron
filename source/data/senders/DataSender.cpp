#include "data/senders/DataSender.hpp"
#include "data/DataPacket.hpp"
#include <cinttypes>
#include <iostream>
#include <string>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

PositionDataSender::PositionDataSender()
{
	StartTick = getTickCount();
}

int64 PositionDataSender::GetTick()
{
	return getTickCount() - StartTick;
}

void PositionDataSender::RegisterTrackedObject(TrackedObject* object)
{
	RegisteredObjects.push_back(object);
}



void PositionDataSender::PrintCSVHeader(ofstream &file)
{
	file << "ms" << ", ";
	for (size_t i = 0; i < RegisteredObjects.size(); i++)
	{
		file << "numeral, X , Y, rot(deg)";
		if (i != RegisteredObjects.size() -1)
		{
			file << ", ";
		}
		
	}
	file << endl;
}

void PositionDataSender::PrintCSV(ofstream &file)
{
	uint32_t ms = (getTickCount() - StartTick) *1000 / getTickFrequency();
	file << ms << ", ";
	for (size_t i = 0; i < RegisteredObjects.size(); i++)
	{
		vector<PositionPacket> packets = RegisteredObjects[i]->ToPacket(i);
		for (size_t j = 0; j < packets.size(); j++)
		{
			file << packets[j].ToCSV();
			if ((i != RegisteredObjects.size() -1) || (j != packets.size() -1))
			{
				file << ", ";
			}
		}
	}
	file << endl;
}