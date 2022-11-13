#pragma once

#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include "TrackedObjects/TrackedObject.hpp"
#include "data/senders/Encoders/GenericEncoder.hpp"
#include "data/senders/Transport/GenericTransport.hpp"

class PositionDataSender
{
protected:
	std::vector<TrackedObject*> RegisteredObjects;

	int64 StartTick;

	std::thread *ReceiveThread;
	std::mutex ReceiveKillMutex;

public:
	GenericEncoder *encoder;
	GenericTransport *transport; 


	PositionDataSender();

	virtual ~PositionDataSender();

	int64 GetTick();

	virtual void RegisterTrackedObject(TrackedObject* object);

	virtual void SendPacket(int64 GrabTick);

	void ThreadRoutine();

	void StartReceiveThread();
	void StopReceiveThread();
};