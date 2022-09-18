#pragma once
#include "data/senders/DataSender.hpp"
#include "thirdparty/serialib.h"


class SerialSender : public PositionDataSender
{
private:
	std::vector<TrackedObject*> SerialObjects;
	serialib* Bridge;

	int64 StartTick;

public:
	SerialSender(bool SelfDetectSerial);
	SerialSender(serialib* InBridge);
	~SerialSender();

	serialib* GetBridge();

	void SendPacket() override;

	static std::vector<cv::String> autoDetectTTYUSB();

};
