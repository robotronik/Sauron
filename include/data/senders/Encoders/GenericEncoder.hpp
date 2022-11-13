#pragma once

#include <vector>
#include <cstdint>

#include "TrackedObjects/ObjectIdentity.hpp"
#include "TrackedObjects/TrackedObject.hpp"

struct DecodedData
{
	int64 GrabTime;
	std::vector<TrackedObject*> objects;
};

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
public:
    GenericEncoder();
    int64 GetTick();
	virtual EncodedData Encode(DecodedData* data) = 0;
};
