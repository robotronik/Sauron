#pragma once
#include "data/senders/DataSender.hpp"
#include "thirdparty/serialib.h"

using namespace std;
using namespace cv;

class SerialSender : public DataSender
{
private:
	vector<TrackedObject*> SerialObjects;
	serialib* Bridge;

	int64 StartTick;

public:
	SerialSender();
	SerialSender(serialib* InBridge);
	~SerialSender();

	void RegisterTrackedObject(TrackedObject* object) override;

	void SendPacket() override;

	static vector<String> autoDetectTTYUSB();

};
