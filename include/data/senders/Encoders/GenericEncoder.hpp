#pragma once

#include <vector>
#include <cstdint>

#include "TrackedObjects/ObjectIdentity.hpp"
#include "TrackedObjects/TrackedObject.hpp"

struct EncodedData
{
	bool valid;
	size_t length;
	char* buffer;

	EncodedData()
	:valid(false), length(0), buffer(nullptr)
	{}

	EncodedData(size_t inlength, char* inbuffer)
	:valid(true), length(inlength), buffer(inbuffer)
	{}
};



class GenericEncoder
{
	int64 CreationTick;
protected:
	uint8_t AllowMask;
public:
    GenericEncoder(uint8_t InAllowMask = UINT8_MAX);
    int64 GetTick();
	virtual EncodedData Encode(int64 GrabTime, std::vector<ObjectData> &objects) = 0;
};
