#include "data/senders/Transport/TCPTransport.hpp"


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

TCPTransport::TCPTransport(bool inServer, string inIP, int inPort, string inInterface)
    : GenericTransport(),
    Server(inServer),
    IP(inIP), Port(inPort), Interface(inInterface)
{	
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == -1)
	{
		cerr << "Failed to create socket" << endl;
	}
	
    if(setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, Interface.c_str(), Interface.size()))
    {
        cerr << "Failed to set socket opt : " << errno << endl;
    }
    struct sockaddr_in serverAddress;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(Port);

    string ip = Server ? "0.0.0.0" : IP;

    if (inet_pton(AF_INET, ip.c_str(), &serverAddress.sin_addr) <= 0) {
        cerr << "ERROR : Invalid address/ Address not supported \n" << endl;
    }

    if (Server)
    {
        cout << "Binding TCP socket..." << endl;
        if (bind(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1) 
        {
            cerr << "Can't bind to IP/port, errno " << errno << endl;
        }
        cout << "Marking TCP socket for listening..." << endl;
        if (listen(sockfd, SOMAXCONN) == -1)
        {
            cerr << "Can't listen !" << endl;
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

TCPTransport::~TCPTransport()
{
    for (int i = 0; i < connectionfd.size(); i++)
    {
        close(connectionfd[i]);
    }
    if (sockfd != -1)
    {
        close(sockfd);
    }
    
}

void TCPTransport::Broadcast(const void *buffer, int length)
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
        for (int i = 0; i < connectionfd.size(); i++)
        {
            int err = send(connectionfd[i], buffer, length, MSG_NOSIGNAL);
            if (err && (errno != EAGAIN && errno != EWOULDBLOCK))
            {
                char buffer[100];
                inet_ntop(AF_INET, &connectionaddresses[i], buffer, sizeof(sockaddr_in));
                if (errno == EPIPE)
                {
                    cout << "Client " << buffer << " disconnected." <<endl;
                    close(connectionfd[i]);
                    connectionfd.erase(next(connectionfd.begin(), i));
                    connectionaddresses.erase(next(connectionaddresses.begin(), i));
                    i--;
                }
                else
                {
                    cerr << "Server failed to send data to client " << buffer << " : " << errno << endl;
                }
            }
        }
	}
	else
	{
		int err = send(sockfd, buffer, length, MSG_NOSIGNAL);
		if (err && (errno != EAGAIN && errno != EWOULDBLOCK))
		{
            if (errno == EPIPE)
            {
                cout << "Server has disconnected" << endl;
                close(sockfd);
                sockfd = -1;
                Connected = false;
                //todo : reconnect
            }
            else
            {
                cerr << "Failed to send data : " << errno << endl;
            }
		}
	}
}

int TCPTransport::Receive(const void *buffer, int maxlength)
{
    return -1;
}

void TCPTransport::receiveThread()
{
	cout << "TCP thread started..." << endl;
	char dataReceived[1025];
	int n;
	while (1)
	{
		this_thread::sleep_for(chrono::milliseconds(10));
		if (Server)
		{
			sockaddr_in client;
			socklen_t clientSize = sizeof(client);
			bzero(&client, clientSize);
            int ret = accept4(sockfd, (struct sockaddr *)&client, &clientSize, 0);
            if (ret > 0)
            {
                char buffer[100];
                inet_ntop(AF_INET, &client, buffer, clientSize);
                cout << "Client connecting from " << buffer << " fd=" << ret << endl;
                unique_lock lock(listenmutex);
                connectionfd.push_back(ret);
                connectionaddresses.push_back(client);
            }
            else 
            {
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    
                }
                else
                {
                    cerr << "Unhandled error on TCP accept: " << errno << endl;
                }
            }
			shared_lock lock(listenmutex);
            for (int i = 0; i < connectionfd.size(); i++)
            {
                memset(dataReceived, '0' ,sizeof(dataReceived));
                while ((n = read(connectionfd[i], dataReceived, sizeof(dataReceived)-1)) > 0)
                {
                    dataReceived[n] = 0;
                    
                    //cout << "Received " << n << " bytes..." << endl;
                    //printBuffer(dataReceived, n);
                }
            }
		}
		else
		{
			memset(dataReceived, '0' ,sizeof(dataReceived));
			while ((n = read(sockfd, dataReceived, sizeof(dataReceived)-1)) > 0)
			{
				dataReceived[n] = 0;
				
				//cout << "Received " << n << " bytes..." << endl;
				//printBuffer(dataReceived, n);
			}
		}
		
		
	}
	
}