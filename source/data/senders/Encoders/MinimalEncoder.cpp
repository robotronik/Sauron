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
	assert(size <= maxlength);
	buffer += identity.PackInto(buffer, maxlength);
	memcpy(buffer, &X, sizeof(float)*3);
	return size;
}

int PositionPacket::UnpackFrom(const char* buffer, int maxlength)
{
	int taken = identity.UnpackFrom(buffer, maxlength);
	assert(maxlength >= taken + 3*sizeof(float));
	memcpy(&X, buffer + taken, sizeof(float)*3);
	taken += sizeof(float)*3;
	return taken;
}

void MinimalEncoder::Affine3dTo2D(PositionPacket &InPacket, Affine3d position)
{
	Vec3d pos3d = position.translation(); //convert to mm
	InPacket.X = pos3d(0);
	InPacket.Y = pos3d(1);
	double angle = GetRotZ(position.linear());
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
			Affine3dTo2D(packet, ObjectDatas[i].location);
			packetsize += packet.GetSize();
			ObjectPackets.push_back(packet);
		}
	}
	
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

int MinimalEncoder::Decode(const EncodedData &data, DecodedMinimalData &outdecoded)
{
	if (data.length < sizeof(MinimalPacketHeader))
	{
		return 0;
	}
	const MinimalPacketHeader* header = reinterpret_cast<MinimalPacketHeader*>(data.buffer);
	if (data.length < header->TotalLength)
	{
		return 0;
	}
	outdecoded.Header = *header;
	outdecoded.Objects.resize(header->NumDatas);
	const char* decodeptr = data.buffer + sizeof(MinimalPacketHeader);
	for (int i = 0; i < header->NumDatas; i++)
	{
		int eaten = decodeptr - data.buffer;
		decodeptr += outdecoded.Objects[i].UnpackFrom(decodeptr, data.length-eaten);
	}
	assert(decodeptr-data.buffer == header->TotalLength);
	return header->TotalLength;
}