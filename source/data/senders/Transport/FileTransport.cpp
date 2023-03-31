#include "data/senders/Transport/FileTransport.hpp"


using namespace std;


FileTransport::FileTransport(string path)
{
	file = make_unique<ofstream>(path);
}

void FileTransport::Broadcast(const void *buffer, int length)
{
	file->write((const char*)buffer, length);
}

int FileTransport::Receive(void *buffer, int maxlength, bool blocking)
{
	return -1;
}