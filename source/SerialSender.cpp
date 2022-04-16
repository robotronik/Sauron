#include "SerialSender.hpp"
#include "math3d.hpp"
#include "filesystem"

namespace fs = std::filesystem;

SerialSender::SerialSender(serialib* InBridge)
{
	Bridge = InBridge;
	StartTick = getTickCount();
}

SerialSender::~SerialSender()
{
	delete Bridge;
}

void SerialSender::RegisterRobot(TrackerCube* tracker)
{
	Robots.push_back(tracker);
}

void SerialSender::SendPacket()
{
	uint32_t ms = (getTickCount() - StartTick) *1000 / getTickFrequency();
	SerialPacketOut packet;
	packet.NumRobots = Robots.size();
	packet.NumPalets = 0; //TODO : track palets
	packet.score = 0; // TODO : compute score
	packet.ms = ms;
	packet.robots.resize(packet.NumRobots);
	packet.palets.resize(packet.NumPalets);
	for (size_t i = 0; i < packet.NumRobots; i++)
	{
		RobotPacket robot = Robots[i]->ToPacket(i);
		packet.robots.push_back(robot);
	}
	
	uint32_t buffsize = packet.GetPacketSize();
	void* buff = packet.ToBuffer();
	if (!Bridge->isDeviceOpen())
	{
		cerr << "Serial bridge not open, cannot send serial data" << endl;
		return;
	}

	
	
	Bridge->writeBytes(buff, buffsize);
}

void SerialSender::PrintCSVHeader(ofstream &file)
{
	file << "ms" << ", ";
	for (size_t i = 0; i < Robots.size(); i++)
	{
		file << "num robot, X , Y, rot(deg)";
		if (i != Robots.size() -1)
		{
			file << ", ";
		}
		
	}
	file << endl;
}

void SerialSender::PrintCSV(ofstream &file)
{
	uint32_t ms = (getTickCount() - StartTick) *1000 / getTickFrequency();
	file << ms << ", ";
	for (size_t i = 0; i < Robots.size(); i++)
	{
		file << Robots[i]->ToPacket(i).ToCSV();
		if (i != Robots.size() -1)
		{
			file << ", ";
		}
		
	}
	file << endl;
}

vector<String> SerialSender::autoDetectTTYUSB()
{
	vector<String> USBtities;
	String target = "ttyUSB";
	for (const auto& entry : fs::directory_iterator("/dev"))
	{
		auto filename = entry.path().filename();
		String filenamestr = filename.string();
		int location = filenamestr.find(target);
		//cout << filename << "@" << location << endl;
		if (location != string::npos)
		{
			USBtities.push_back(entry.path().string());
		}
	}
	return USBtities;
}