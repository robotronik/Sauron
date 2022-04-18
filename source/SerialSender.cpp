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
	SerialTransmission packet;
	packet.NumPositions = Robots.size();
	packet.score = 0; // TODO : compute score
	packet.ms = ms;
	packet.PositionPackets.resize(packet.NumPositions);
	for (size_t i = 0; i < packet.NumPositions; i++)
	{
		PositionPacket robot = Robots[i]->ToPacket(i);
		packet.PositionPackets[i] = robot;
	}
	
	uint32_t buffsize = packet.GetPacketSize();
	/*cout << "Sending packet of size " << buffsize 
	<< ", total size " 
	<< buffsize + SerialTransmission::StartFlag.size() + SerialTransmission::StopFlag.size()
	<< endl;*/

	void* buff = packet.ToBuffer();
	if (!Bridge->isDeviceOpen())
	{
		//cerr << "Serial bridge not open, cannot send serial data" << endl;
		return;
	}

	
	Bridge->writeBytes(SerialTransmission::StartFlag.data(), SerialTransmission::StartFlag.size());
	Bridge->writeBytes(buff, buffsize);
	Bridge->writeBytes(SerialTransmission::StopFlag.data(), SerialTransmission::StopFlag.size());
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