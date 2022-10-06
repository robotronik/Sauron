#pragma once

#include "data/senders/DataSender.hpp"

#include <nng/nng.h>

class WebSender : public PositionDataSender
{
private:
	nng_socket nngsock;
	nng_listener nnglistener;
	int sockfd;
	vector<int> connectionfd;
	bool connected = false;
public:
	WebSender();
	~WebSender();

	void SendPacket() final;
};
