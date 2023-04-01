#include "data/senders/Transport/FileTransport.hpp"

#include <chrono>
#include <ctime>

using namespace std;


FileTransport::FileTransport(string path)
{
	file = make_unique<ofstream>(path, ios_base::openmode::_S_app | ios_base::openmode::_S_bin | ios_base::openmode::_S_ate);
	auto currtime = chrono::system_clock::now();
	std::time_t curr_time = std::chrono::system_clock::to_time_t(currtime);
	*file << "New run : " << ctime(&curr_time) <<endl;
}

void FileTransport::Broadcast(const void *buffer, int length)
{
	file->write((const char*)buffer, length);
}

int FileTransport::Receive(void *buffer, int maxlength, bool blocking)
{
	return -1;
}