#include "data/senders/WebSender.hpp"

#include "data/DataPacket.hpp"
#include "GlobalConf.hpp"

#include <iostream>

#include "data/senders/Transport/TCPTransport.hpp"
#include "data/senders/Transport/UDPTransport.hpp"

using namespace std;

WebSender::WebSender()
{
	WebsocketConfig config = GetWebsocketConfig();

	if (config.TCP)
	{
		TransportLayer = new TCPTransport(config.Server, config.IP, config.Port, config.Interface);
	}
	else
	{
		TransportLayer = new UDPTransport(config.Server, config.IP, config.Port, config.Interface);
	}
}

WebSender::~WebSender()
{
	delete TransportLayer;
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

	if (false)
	{
		int buffersize = sizeof(int64) + sizeof(uint32_t) + sizeof(PositionPacket) * packets.size();
		char* buffer = reinterpret_cast<char*>(malloc(buffersize));
		char* currptr = buffer;
		*reinterpret_cast<int64*>(currptr) = GetTick();
		currptr += sizeof(int64);
		*reinterpret_cast<uint32_t*>(currptr) = (uint32_t)packets.size();
		currptr += sizeof(uint32_t);
		std::uninitialized_copy(packets.begin(), packets.end(), reinterpret_cast<PositionPacket*>(currptr));

		TransportLayer->Broadcast(buffer, buffersize);
		free(buffer);
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
	
		//cout << "Sending " << outp << endl;
		TransportLayer->Broadcast(outp.c_str(), outp.length());
	}
}