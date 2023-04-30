#include "data/senders/Transport/TCPTransport.hpp"


#include <iostream>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/un.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "GlobalConf.hpp"

using namespace std;

TCPTransport::TCPTransport(bool inServer, string inIP, int inPort, string inInterface)
	: GenericTransport()
{	
	Server = inServer;
	IP = inIP;
	Port = inPort;
	Interface = inInterface;
	sockfd = -1;
	Connected = false;
	CreateSocket();
	Connect();

	ReceiveThreadHandle = new thread(&TCPTransport::receiveThread, this);
}

TCPTransport::~TCPTransport()
{
	cout << "Destroying TCP transport " << IP << ":" << Port << " @ " << Interface <<endl;
	for (int i = 0; i < connectionfd.size(); i++)
	{
		shutdown(connectionfd[i], SHUT_RDWR);
		close(connectionfd[i]);
	}
	if (sockfd != -1)
	{
		shutdown(sockfd, SHUT_RDWR);
		close(sockfd);
	}
}

void TCPTransport::CreateSocket()
{
	if (sockfd != -1)
	{
		return;
	}
	int type = Server ? SOCK_STREAM : SOCK_STREAM | SOCK_NONBLOCK;
	sockfd = socket(AF_INET, type, 0);
	if (sockfd == -1)
	{
		cerr << "TCP Failed to create socket, port " << Port << endl;
	}
	
	if(setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, Interface.c_str(), Interface.size()))
	{
		cerr << "TCP Failed to bind to interface : " << errno << endl;
	}
	LowerLatency(sockfd);
	
}

bool TCPTransport::Connect()
{
	struct sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(Port);

	string ip = Server ? "0.0.0.0" : IP;

	if (inet_pton(AF_INET, ip.c_str(), &serverAddress.sin_addr) <= 0) {
		cerr << "TCP ERROR : Invalid address/ Address not supported \n" << endl;
	}

	if (Server)
	{
		//cout << "TCP Binding socket..." << endl;
		if (bind(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1) 
		{
			cerr << "TCP Can't bind to IP/port, errno " << errno << endl;
		}
		//cout << "TCP Marking socket for listening" << endl;
		if (listen(sockfd, SOMAXCONN) == -1)
		{
			cerr << "TCP Can't listen !" << endl;
		}
		Connected = true;
		return true;
	}
	else
	{
		// communicates with listen
		if(connect(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
		{
			//cerr << "Failed to connect to server" << endl;
			return false;
		}
		else
		{
			cout << "TCP connected to server" << endl;
			Connected = true;
			return true;
		}
		
	}
}

void TCPTransport::CheckConnection()
{
	if (sockfd == -1)
	{
		CreateSocket();
	}
	if (!Connected)
	{
		Connect();
	}
}

void TCPTransport::LowerLatency(int fd)
{
	int corking = 0;
	if (setsockopt(fd, IPPROTO_TCP, TCP_CORK, &corking, sizeof(corking)))
	{
		cerr << "TCP Failed to disable corking : " << errno << endl;
	}
	int nodelay = 1;
	if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay)))
	{
		cerr << "TCP Failed to disable corking : " << errno << endl;
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
	if (length > 1000)
	{
		cerr << "WARNING : Packet length over 1000, packet may be dropped" << endl;
	}

	if (Server)
	{
		shared_lock lock(listenmutex);
		for (int i = 0; i < connectionfd.size(); i++)
		{
			int err = send(connectionfd[i], buffer, length, MSG_NOSIGNAL);
			int errnocp = errno;
			if (err ==-1 && (errnocp != EAGAIN && errnocp != EWOULDBLOCK))
			{
				char buffer[100];
				inet_ntop(AF_INET, &connectionaddresses[i].sin_addr, buffer, sizeof(sockaddr_in));
				if (errnocp == EPIPE)
				{
					cout << "TCP Client " << buffer << " disconnected." <<endl;
					close(connectionfd[i]);
					connectionfd.erase(next(connectionfd.begin(), i));
					connectionaddresses.erase(next(connectionaddresses.begin(), i));
					i--;
				}
				else
				{
					cerr << "TCP Server failed to send data to client " << buffer << " : " << errnocp << endl;
				}
			}
		}
	}
	else
	{
		int err = send(sockfd, buffer, length, MSG_NOSIGNAL);
		int errnocp = errno;
		if (err ==-1 && (errnocp != EAGAIN && errnocp != EWOULDBLOCK))
		{
			if (errnocp == EPIPE)
			{
				cout << "TCP Server has disconnected" << endl;
				close(sockfd);
				sockfd = -1;
				Connected = false;
			}
			else
			{
				cerr << "TCP Failed to send data : " << errnocp << endl;
			}
		}
	}
}

int TCPTransport::Receive(void *buffer, int maxlength, bool blocking)
{
	int n = -1;
	memset(buffer, '0' , maxlength);
	int flags = blocking ? 0 : MSG_DONTWAIT;

	if (Server)
	{
		shared_lock lock(listenmutex);
		for (int i = 0; i < connectionfd.size(); i++)
		{
			if ((n = recv(connectionfd[i], buffer, maxlength, MSG_DONTWAIT)) > 0) //cannot wait because server
			{
				//cout << "Received " << n << " bytes..." << endl;
				//printBuffer(dataReceived, n);
				return n;
			}
		}
	}
	else
	{
		if ((n = recv(sockfd, buffer, maxlength, flags)) > 0)
		{
			//cout << "Received " << n << " bytes..." << endl;
			//printBuffer(dataReceived, n);
			return n;
		}
	}
	return -1;
}

void TCPTransport::receiveThread()
{
	cout << "TCP Webserver thread started..." << endl;
	int n;
	while (1)
	{
		this_thread::sleep_for(chrono::milliseconds(100));
		CheckConnection();
		if (Server)
		{
			sockaddr_in client;
			socklen_t clientSize = sizeof(client);
			bzero(&client, clientSize);
			int ret = accept4(sockfd, (struct sockaddr *)&client, &clientSize, 0);
			if (ret > 0)
			{
				LowerLatency(ret);
				char buffer[100];
				inet_ntop(AF_INET, &client.sin_addr, buffer, clientSize);
				cout << "TCP Client connecting from " << buffer << " fd=" << ret << endl;
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
					cerr << "TCP Unhandled error on accept: " << errno << endl;
				}
			}
			
		}
	}
	
}