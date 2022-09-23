#pragma once

#include "data/senders/DataSender.hpp"

class WebSender : public PositionDataSender
{
private:
	int sockfd;
	bool connected = false;
public:
	WebSender(bool local, bool TCP);
	~WebSender();

	void SendPacket() final;
};
