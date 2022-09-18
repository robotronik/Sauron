#pragma once

#include <vector>
#include <fstream>
#include "TrackedObjects/TrackedObject.hpp"


class PositionDataSender
{
protected:
	std::vector<TrackedObject*> SerialObjects;

	int64 StartTick;

public:
	PositionDataSender();

	virtual ~PositionDataSender(){};

	virtual void RegisterTrackedObject(TrackedObject* object);

	virtual void SendPacket() = 0;

	virtual void PrintCSVHeader(std::ofstream &file);

	virtual void PrintCSV(std::ofstream &file);
};