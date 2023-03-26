#pragma once

#include "data/senders/Encoders/GenericEncoder.hpp"

#include <map>

struct __attribute__ ((packed)) MinimalPacketHeader
{
	uint32_t TotalLength; //length of the full transaction
	uint64_t SentTick; //tick at which the transaction was sent
	uint64_t Latency; //ticks it took to process the images from the cameras
	uint32_t TickRate; //ticks/s
	uint32_t NumDatas; //number of PositionPackets
};


struct PositionPacket
{
	ObjectIdentity identity;
	float X;
	float Y;
	float rotation; //-pi to pi

	PositionPacket()
	:identity(), X(0), Y(0), rotation(0)
	{};

	PositionPacket(ObjectIdentity InIdentity, float InX, float InY, float InRotation)
	:identity(InIdentity), X(InX), Y(InY), rotation(InRotation)
	{};

	size_t GetSize() const
	{
		return identity.GetSize() + sizeof(float)*3;
	};

	int PackInto(char* buffer, int maxlength) const;
};

//An encoder that only sends X, Y and rot according to the struct above.
//Position is in "Victor" coordinates (mm, degrees)
class MinimalEncoder : public GenericEncoder
{
protected:
	std::unordered_map<PacketType, bool> AllowMask;
public:
	MinimalEncoder(std::unordered_map<PacketType, bool> InAllowMask)
		:GenericEncoder(), AllowMask(InAllowMask)
	{}

	static void Affine3dToVictor(PositionPacket &InPacket, cv::Affine3d position);

	virtual EncodedData Encode(int64 GrabTime, std::vector<ObjectData> &objects) override;
};
