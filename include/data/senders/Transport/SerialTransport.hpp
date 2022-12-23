#pragma once

#include "data/senders/Transport/GenericTransport.hpp"
#include "thirdparty/serialib.h"

#include <vector>
#include <cstring>

//Serial transport layer with SLIP encoding

class SerialTransport : public GenericTransport
{
private:
	serialib* Bridge;
    bool Connected;
public:

    static std::vector<std::string> autoDetectTTYUSB();

	SerialTransport(unsigned int BaudRate, bool SelfDetectSerial);

	virtual void Broadcast(const void *buffer, int length) override;

	virtual int Receive(void *buffer, int maxlength, bool blocking=false) override;
};