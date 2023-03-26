#include "data/senders/Encoders/MinimalEncoder.hpp"

#include <sstream>
#include <cassert>

#include <opencv2/core.hpp>

#include "math3d.hpp"
#include "TrackedObjects/ObjectIdentity.hpp"

using namespace std;
using namespace cv;

int PositionPacket::PackInto(char* buffer, int maxlength) const
{
	int size = GetSize();
	assert(size >= maxlength);
	buffer += identity.PackInto(buffer, maxlength);
	memcpy(buffer, &X, sizeof(float)*3);
	return size;
}

void MinimalEncoder::Affine3dToVictor(PositionPacket &InPacket, Affine3d position)
{
	Vec3d pos3d = position.translation() * 1000.0; //convert to mm
	InPacket.X = pos3d(0);
	InPacket.Y = pos3d(1);
	double angle = GetRotZ(position.linear()) * 180.f / M_PI;
	InPacket.rotation = angle;
}

EncodedData MinimalEncoder::Encode(int64 GrabTime, std::vector<ObjectData> &objects)
{
	vector<ObjectData>& ObjectDatas = objects;

	vector<PositionPacket> ObjectPackets;
	ObjectPackets.reserve(ObjectDatas.size());
	size_t packetsize = 0;

	for (size_t i = 0; i < ObjectDatas.size(); i++)
	{
		if (AllowMask[ObjectDatas[i].identity.type])
		{
			PositionPacket packet;
			packet.identity = ObjectDatas[i].identity;
			Affine3dToVictor(packet, ObjectDatas[i].location);
			packetsize += packet.GetSize();
			ObjectPackets.push_back(packet);
		}
	}
	
	if (true)
	{
		size_t buffersize = sizeof(MinimalPacketHeader) + packetsize;
		char* buffer = reinterpret_cast<char*>(malloc(buffersize));
		char* endptr = buffer + buffersize;
		char* currptr = buffer + sizeof(MinimalPacketHeader);
		for (int i = 0; i < ObjectPackets.size(); i++)
		{
			currptr += ObjectPackets[i].PackInto(currptr, endptr - currptr);
		}
		
		MinimalPacketHeader* header = reinterpret_cast<MinimalPacketHeader*>(buffer);
		header->NumDatas = ObjectPackets.size();
		header->TickRate = cv::getTickFrequency();
		header->SentTick = cv::getTickCount();
		header->Latency = header->SentTick - GrabTime;
		header->TotalLength = buffersize;
		//memcpy(buffer, &header, sizeof(MinimalPacketHeader));
        return EncodedData(buffersize, buffer);
	}
	else
	{
		ostringstream sendbuff;
		MinimalPacketHeader header;
		header.NumDatas = ObjectPackets.size();
		header.TickRate = cv::getTickFrequency();
		header.SentTick = cv::getTickCount();
		header.Latency = header.SentTick - GrabTime;

		sendbuff << header.SentTick << "," << header.Latency << "," << header.TickRate << "," << header.NumDatas << "{";

		for (int i = 0; i < ObjectPackets.size(); i++)
		{
			PositionPacket& packet = ObjectPackets[i];
			sendbuff << (int)packet.identity.type << "," << (int)packet.identity.numeral << "," << packet.X << "," << packet.Y << "," << packet.rotation << ";";
		}

		if (ObjectPackets.size() > 0)
		{
			int pos = sendbuff.tellp();
			sendbuff.seekp(pos-1);//remove trailing ;
		}
		sendbuff << "}";
		string outp = sendbuff.str();
		//cout << "sending " << outp << endl;
		char* buffer = (char*)malloc(outp.size()+1);
		memcpy(buffer, outp.c_str(), outp.size()+1);
        return EncodedData(outp.size()+1, buffer);
	}
}