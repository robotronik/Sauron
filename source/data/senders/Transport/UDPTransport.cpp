#include "data/senders/Transport/UDPTransport.hpp"

#include <iostream>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "GlobalConf.hpp"

using namespace std;

UDPTransport::UDPTransport(bool inServer, std::string inIP, int inPort, std::string inInterface)
    :GenericTransport(),
    Server(inServer),
    IP(inIP), Port(inPort), Interface(inInterface)
{
    WebsocketConfig config = GetWebsocketConfig();

	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if (sockfd == -1)
	{
		cerr << "Failed to create socket" << endl;
	}

    string interface = "eth1";
    setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, interface.c_str(), interface.size() );
    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(config.Port);

    string ip = config.Server ? "0.0.0.0" : config.IP;

    if (inet_pton(AF_INET, ip.c_str(), &serverAddress.sin_addr) <= 0) {
        cerr << "ERROR : Invalid address/ Address not supported \n" << endl;
    }

    if (Server)
    {
        cout << "Binding UDP socket..." << endl;
        if (bind(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1) 
        {
            cerr << "Can't bind to IP/port, errno " << errno << endl;
        }
        Connected = true;
        ReceiveThreadHandle = new thread([this](){receiveThread();});
    }
    else
    {
        // communicates with listen
        if(connect(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
        {
            cerr << "Failed to connect to server" << endl;
        }
        else
        {
            Connected = true;
            ReceiveThreadHandle = new thread([this](){receiveThread();});
        }
        
    }
}

void UDPTransport::Broadcast(const void *buffer, int length)
{
	if (!Connected)
	{
		return;
	}
	//cout << "Sending " << length << " bytes..." << endl;
	//printBuffer(buffer, length);
	if (Server)
	{
		shared_lock lock(listenmutex);
        for (int i = 0; i < connectionaddresses.size(); i++)
        {
            int err = sendto(sockfd, buffer, length, 0, (struct sockaddr*)&connectionaddresses[i], sizeof(sockaddr_in));
            if (err && (errno != EAGAIN && errno != EWOULDBLOCK))
            {
                cerr << "Server failed to send data to client " << i << " : " << errno << endl;
            }
        }
	}
	else
	{

		int err = send(sockfd, buffer, length, 0);
		if (err && (errno != EAGAIN && errno != EWOULDBLOCK))
		{
			cerr << "Failed to send data : " << errno << endl;
		}
	}
}

int UDPTransport::Receive(const void *buffer, int maxlength)
{
    return -1;
}
	
void UDPTransport::receiveThread()
{
	cout << "Webserver thread started..." << endl;
	char dataReceived[1024];
	int n;
	while (1)
	{
		this_thread::sleep_for(chrono::milliseconds(10));
        
        sockaddr_in client;
        socklen_t clientSize = sizeof(client);
        bzero(&client, clientSize);
        shared_lock lock(listenmutex);
        while ((n = recvfrom(sockfd, dataReceived, sizeof(dataReceived)-1, 0, (struct sockaddr*)&client, &clientSize)) > 0)
        {
            bool found = false;
            for (int i = 0; i < connectionaddresses.size(); i++)
            {
                if (connectionaddresses[i].sin_addr.s_addr == client.sin_addr.s_addr)
                {
                    found = true;
                    break;
                }
                
            }
            if (!found)
            {
                connectionaddresses.push_back(client);
                char buffer[100];
                inet_ntop(AF_INET, &client, buffer, clientSize);
                cout << "Client connecting from " << buffer << endl;
            }
            dataReceived[n] = 0;
                
            //cout << "Received " << n << " bytes..." << endl;
            //printBuffer(dataReceived, n);
        }
	}
	
}