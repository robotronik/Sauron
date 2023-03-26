#include "TrackedObjects/ObjectIdentity.hpp"
#include <cassert>

int ObjectIdentity::PackInto(char* buffer, int maxlength) const
{
	int size = GetSize();
	assert(size <= maxlength);
	
	PackedIdentity ph(*this);
	memcpy(buffer, &ph, sizeof(ph));
	buffer += sizeof(ph);
	memcpy(buffer, metadata.data(), metadata.size());
	return size;
}