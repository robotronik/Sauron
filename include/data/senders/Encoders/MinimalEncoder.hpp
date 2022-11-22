#pragma once

#include "data/senders/Encoders/GenericEncoder.hpp"

struct __attribute__ ((packed)) MinimalPacketHeader
{
	int64_t SentTick;
	int64_t Latency;
	int32_t TickRate;
	int32_t NumDatas;
};


struct __attribute__ ((packed)) PositionPacket //palets sur l'arène uniquement
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
};

class MinimalEncoder : public GenericEncoder
{
public:
	MinimalEncoder(uint8_t InAllowMask = UINT8_MAX)
		:GenericEncoder(InAllowMask)
	{}

	static void Affine3dToVictor(PositionPacket &InPacket, cv::Affine3d position);

	virtual EncodedData Encode(int64 GrabTime, std::vector<ObjectData> &objects) override;
};
