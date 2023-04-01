#pragma once

#include <map>

#include "data/senders/Encoders/GenericEncoder.hpp"



class TextEncoder : public GenericEncoder
{
protected:
	std::map<PacketType, bool> AllowMask;
public:
	TextEncoder(std::map<PacketType, bool> InAllowMask)
		:GenericEncoder(), AllowMask(InAllowMask)
	{}

	virtual EncodedData Encode(int64 GrabTime, std::vector<ObjectData> &objects) override;
};