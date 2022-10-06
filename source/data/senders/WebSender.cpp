#include "data/senders/WebSender.hpp"

#include "data/DataPacket.hpp"
#include "GlobalConf.hpp"

#include <filesystem>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <nng/nng.h>
#include <nng/protocol/pipeline0/pull.h>
#include <nng/protocol/pipeline0/push.h>



#define rcvbufsze 1024

using namespace std;

WebSender::WebSender()
{
	WebsocketConfig config = GetWebsocketConfig();
	int rv;
	if(rv = nng_push0_open(&nngsock))
	{
		cerr << "Failed to open nng socket, err" << rv << endl;
	}

	if (config.Server)
	{
		if(rv = nng_listen(nngsock, config.IP.c_str(), &nnglistener, 0))
		{
			cerr << "Failed to bind to socket, " << nng_strerror(rv) << endl;
		}
	}
	
	

	for (int i = 0; i < config.URLs.size(); i++)
	{
		if (rv = nng_dial(nngsock, config.URLs[i].c_str(), NULL, 0))
		{
			cerr << "Failed to dial URL " << i << " : \"" << config.URLs[i] << "\", err" << rv << endl;
		}
	}
	

	/*
	bool &local = config.Unix;
	bool &TCP = config.TCP;
	bool &Server = config.Server;
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


		if (inet_pton(AF_INET, config.IP.c_str(), &serverAddress.sin_addr) <= 0) {
			cerr << "ERROR : Invalid address/ Address not supported \n" << endl;
    	}
		// communicates with listen
		connect(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
	}*/
	
	
	
	/*
	char serverResponse[rcvbufsze];
	recv(sockfd, &serverResponse, sizeof(serverResponse), 0);
	printf("Received data from server : %s", serverResponse);*/

	//closing the socket
	
}

WebSender::~WebSender()
{
	nng_close(nngsock);
	//close(sockfd);
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
		char* buffer = reinterpret_cast<char*>(malloc(buffersize));
		char* currptr = buffer;
		*reinterpret_cast<int64*>(currptr) = GetTick();
		currptr += sizeof(int64);
		*reinterpret_cast<uint32_t*>(currptr) = (uint32_t)packets.size();
		currptr += sizeof(uint32_t);
		std::uninitialized_copy(packets.begin(), packets.end(), reinterpret_cast<PositionPacket*>(currptr));

		//send(sockfd, buffer, buffersize, 0);
		if (nng_send(nngsock, buffer, buffersize, NNG_FLAG_ALLOC | NNG_FLAG_NONBLOCK))
		{
			cerr << "Failed to send data" << endl;
		}
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