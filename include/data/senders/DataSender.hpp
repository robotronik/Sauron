#pragma once

#include <vector>
#include <fstream>
#include "TrackedObjects/TrackedObject.hpp"

using namespace std;

class PositionDataSender
{
protected:
	vector<TrackedObject*> SerialObjects;

	int64 StartTick;

public:
	PositionDataSender();

	virtual ~PositionDataSender(){};

	virtual void RegisterTrackedObject(TrackedObject* object);

	virtual void SendPacket() = 0;

	virtual void PrintCSVHeader(ofstream &file);

	virtual void PrintCSV(ofstream &file);
};