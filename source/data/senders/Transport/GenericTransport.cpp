#include "data/senders/Transport/GenericTransport.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

void GenericTransport::printBuffer(const void* buffer, int length)
{
	ostringstream stream;
	stream << std::setfill ('0') << std::setw(3) 
		<< std::hex;
	int i;
	for (i = 0; i < 8; i++)
	{
		stream << std::setw(2) << (unsigned int)i << " ";
	}
	
	for (i = 0; i < length; i++)
	{
		if (i%8 == 0)
		{
			stream << endl;
		}
		stream << std::setw(2) << (unsigned int)((uint8_t*)buffer)[i] << " ";
	}
	stream << endl;
	
	cout << stream.str();
}

void GenericTransport::Broadcast(const void *buffer, int length)
{
	cerr << "Called Broadcast on base transport class" << endl;
}

int GenericTransport::Receive(const void *buffer, int maxlength)
{
	cerr << "Called receive on base transport class" << endl;
	return -1;
}