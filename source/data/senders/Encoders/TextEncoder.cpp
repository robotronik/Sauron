#include "data/senders/Encoders/TextEncoder.hpp"

#include <sstream>
using namespace std;

EncodedData TextEncoder::Encode(int64 GrabTime, vector<ObjectData> &objects)
{
	ostringstream outstream;
	int numallowed = 0;
	outstream << "Tick " << GrabTime << endl;
	for (size_t i = 0; i < objects.size(); i++)
	{
		ObjectData& object = objects[i];
		if (!AllowMask[object.identity.type])
		{
			continue;
		}
		outstream << "Object[" << i << "]: type=" << (int) object.identity.type << ", numeral=" << (int)object.identity.numeral << " metadata(" << object.identity.metadata.length() << ")=";
		for (int j = 0; j < object.identity.metadata.length(); j++)
		{
			outstream << (int) object.identity.metadata[j] << " ";
 		}
		outstream << ", location=" << endl;
		outstream << object.location.matrix;
		outstream << endl;
		numallowed++;
	}
	outstream << endl;
	string outp;
	if (numallowed == 0)
	{
		return EncodedData(0, nullptr);
	}
	else
	{
		outp = outstream.str();
	}
	char* buffer = (char*)malloc(outp.size());
	memcpy(buffer, outp.c_str(), outp.size());
	//cout << outp <<endl;
	return EncodedData(outp.size(), buffer);
}