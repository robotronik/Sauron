#include "data/senders/WebSender.hpp"

#include "data/DataPacket.hpp"

#include <filesystem>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <sys/types.h>
#include <unistd.h>

#define rcvbufsze 1024

using namespace std;

WebSender::WebSender(bool local, bool TCP)
{
	sockfd = socket(local ? AF_UNIX : AF_INET, TCP ? SOCK_STREAM : SOCK_DGRAM, 0);
	// server address
	if (local)
	{
		struct sockaddr_un serverAddress;
		serverAddress.sun_family = AF_UNIX;
		string path = filesystem::absolute("websocket");
		cout << "Websocket connected to \"" << path << "\"" << endl;
		path.copy(serverAddress.sun_path, sizeof(serverAddress.sun_path));

		connect(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
	}
	else
	{
		struct sockaddr_in serverAddress;
		serverAddress.sin_family = AF_INET;
		serverAddress.sin_port = htons(9002);
		serverAddress.sin_addr.s_addr = INADDR_ANY;

		// communicates with listen
		connect(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
	}
	
	
	
	/*
	char serverResponse[rcvbufsze];
	recv(sockfd, &serverResponse, sizeof(serverResponse), 0);
	printf("Received data from server : %s", serverResponse);*/

	//closing the socket
	
}

WebSender::~WebSender()
{
	close(sockfd);
}

void WebSender::SendPacket()
{
	

	vector<PositionPacket> packets;

	for (int i = 0; i < RegisteredObjects.size(); i++)
	{
		vector<PositionPacket> lp = RegisteredObjects[i]->ToPacket(i);
		packets.insert(packets.end(), lp.begin(), lp.end());
		
	}

	ostringstream sendbuff;

	if (true)
	{
		int buffersize = sizeof(int64) + sizeof(uint32_t) + sizeof(PositionPacket) * packets.size();
		char* buffer = new char[buffersize];
		char* currptr = buffer;
		*reinterpret_cast<int64*>(currptr) = GetTick();
		currptr += sizeof(int64);
		*reinterpret_cast<uint32_t*>(currptr) = (uint32_t)packets.size();
		currptr += sizeof(uint32_t);
		std::uninitialized_copy(packets.begin(), packets.end(), reinterpret_cast<PositionPacket*>(currptr));

		send(sockfd, buffer, buffersize, 0);
		delete[] buffer;
	}
	else
	{
		sendbuff << GetTick() << "," << packets.size() << "{";

		for (int i = 0; i < packets.size(); i++)
		{
			PositionPacket& packet = packets[i];
			sendbuff << (int)packet.type << "," << (int)packet.numeral << "," << packet.X << "," << packet.Y << "," << packet.rotation << ";";
		}

		if (packets.size() > 0)
		{
			int pos = sendbuff.tellp();
			sendbuff.seekp(pos-1);//remove trailing ;
		}
		sendbuff << "}";
		string outp = sendbuff.str();
	
		cout << outp << endl;
		send(sockfd, outp.c_str(), outp.length()+1, 0);
	}

	
}