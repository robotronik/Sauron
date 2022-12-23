#pragma once

#include "data/senders/Transport/GenericTransport.hpp"

#include <thread>
#include <shared_mutex>
#include <vector>
#include <netinet/in.h>

//TCP transport layer

class TCPTransport : public GenericTransport
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
	std::vector<int> connectionfd;
public:

	TCPTransport(bool inServer, std::string inIP, int inPort, std::string inInterface);

	~TCPTransport();

private:
	void CreateSocket();
	bool Connect();
	void CheckConnection();
	void LowerLatency(int fd);
public:

	virtual void Broadcast(const void *buffer, int length) override;

	virtual int Receive(void *buffer, int maxlength, bool blocking=false) override;
	
	void receiveThread();
};
