#pragma once

#include "data/senders/Transport/GenericTransport.hpp"

#include <thread>
#include <shared_mutex>
#include <vector>
#include <netinet/in.h>

//UDP transport layer

class UDPTransport : public GenericTransport
{
private:
	bool Server;
	std::string IP, Interface;
	int Port;
	int sockfd;
	bool Connected;
	std::thread* ReceiveThreadHandle;
	std::shared_mutex listenmutex;
	std::vector<sockaddr_in> connectionaddresses;
public:

	UDPTransport(bool inServer, std::string inIP, int inPort, std::string inInterface);

	~UDPTransport();

	virtual void Broadcast(const void *buffer, int length) override;

	virtual int Receive(void *buffer, int maxlength) override;
	
	void receiveThread();
};
