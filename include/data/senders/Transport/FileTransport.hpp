#pragma once

#include "data/senders/Transport/GenericTransport.hpp"

#include <vector>
#include <cstring>
#include <fstream>
#include <memory>

//Saves everything into a file

class FileTransport : public GenericTransport
{
private:
	std::unique_ptr<std::ofstream> file;
public:

	FileTransport(std::string path);

	virtual void Broadcast(const void *buffer, int length) override;

	virtual int Receive(void *buffer, int maxlength, bool blocking=false) override;
};