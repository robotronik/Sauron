#include "data/metadata.hpp"


std::string MakeTag(float SideLength, int Number)
{
	std::string meta = "";
	AddTypeToMetadata(meta, SideLength);
	AddTypeToMetadata<uint8_t>(meta, Number);
	return meta;
}

bool GetTag(std::string meta, float& SideLength, int Number)
{
	if (meta.length() != sizeof(float) + sizeof(uint8_t))
	{
		return false;
	}
	SideLength = GetTypeFromMetadata<float>(meta, 0);
	Number = GetTypeFromMetadata<uint8_t>(meta, sizeof(float));
	return true;
}