#pragma once

#include <string>

template<class T>
T& GetTypeFromMetadata(std::string &data, size_t pos)
{
	return *(T*)&data[pos];
};

template<class T>
void AddTypeToMetadata(std::string &data, T value)
{
	std::size_t pos = data.size();
	data.resize(pos + sizeof(T));
	GetTypeFromMetadata<T>(data, pos) = value;
};

std::string MakeTag(float SideLength, int Number);

bool GetTag(std::string meta, float& SideLength, int Number);