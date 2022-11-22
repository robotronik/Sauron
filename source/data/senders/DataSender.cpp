#include "data/senders/DataSender.hpp"
#include <cinttypes>
#include <iostream>
#include <string>
#include <opencv2/core.hpp>

using namespace cv;
using namespace std;

PositionDataSender::PositionDataSender()
	:encoder(nullptr), transport(nullptr), ReceiveThread(nullptr)
{
	StartTick = getTickCount();
}

PositionDataSender::~PositionDataSender()
{
	if (ReceiveThread != nullptr)
	{
		StopReceiveThread();
	}
	
	if (encoder != nullptr)
	{
		delete encoder;
	}
	if (transport != nullptr)
	{
		delete transport;
	}
}

int64 PositionDataSender::GetTick()
{
	return getTickCount() - StartTick;
}

void PositionDataSender::SendPacket(int64 GrabTick, vector<ObjectData> &Data)
{
	if (encoder == nullptr)
	{
		cerr << "PDS Encoder is null, no data will be sent." << endl;
		return;
	}
	if (transport == nullptr)
	{
		cerr << "PDS transport is null, no data will be sent." << endl;
		return;
	}
	EncodedData encoded = encoder->Encode(GrabTick, Data);
	if (!encoded.valid)
	{
		cerr << "PDS encoded data is invalid, no data will be sent." << endl;
		return;
	}
	transport->Broadcast(encoded.buffer, encoded.length);
	free(encoded.buffer);
}

void PositionDataSender::ThreadRoutine()
{
	while (1)
	{
		this_thread::sleep_for(chrono::milliseconds(100));
		if (!ReceiveKillMutex.try_lock())
		{
			cout << "Killing PDS receive thread";
			break;
		}
		
		if (transport == nullptr)
		{
			continue;
		}
		char buff[1024];
		int n;
		while((n = transport->Receive(buff, sizeof(buff)))>-1)
		{
			GenericTransport::printBuffer(buff, n);
			//string bufs(buff, n);
			//cout << "Received string \"" << bufs << "\"" << endl;
		}
		ReceiveKillMutex.unlock();
	}
	
}

void PositionDataSender::StartReceiveThread()
{
	if (ReceiveThread != nullptr)
	{
		return;
	}
	ReceiveKillMutex.unlock();
	ReceiveThread = new thread([this](){ThreadRoutine();});
}

void PositionDataSender::StopReceiveThread()
{
	ReceiveKillMutex.lock();
}
