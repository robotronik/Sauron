#pragma once

#include "data/senders/DataSender.hpp"
#include "data/senders/Transport/GenericTransport.hpp"
#include <thread>
#include <shared_mutex>

class WebSender : public PositionDataSender
{
private:
	GenericTransport* TransportLayer;
public:
	WebSender();
	~WebSender();

	void ServerListen();
	
public:

	void SendPacket() final;
};
