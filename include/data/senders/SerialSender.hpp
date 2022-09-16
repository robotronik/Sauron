#pragma once
#include "data/senders/DataSender.hpp"
#include "thirdparty/serialib.h"

using namespace std;
using namespace cv;

class SerialSender : public PositionDataSender
{
private:
	vector<TrackedObject*> SerialObjects;
	serialib* Bridge;

	int64 StartTick;

public:
	SerialSender(bool SelfDetectSerial);
	SerialSender(serialib* InBridge);
	~SerialSender();

	serialib* GetBridge();

	void SendPacket() override;

	static vector<String> autoDetectTTYUSB();

};
