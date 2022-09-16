#pragma once

#include <vector>
#include <fstream>
#include "TrackedObjects/TrackedObject.hpp"

using namespace std;

class DataSender
{
private:
	vector<TrackedObject*> SerialObjects;

	int64 StartTick;

public:
	DataSender();
	~DataSender();

	virtual void RegisterTrackedObject(TrackedObject* object);

	virtual void SendPacket();

	virtual void PrintCSVHeader(ofstream &file);

	virtual void PrintCSV(ofstream &file);
};