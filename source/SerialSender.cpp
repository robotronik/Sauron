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

void RegisterTrackedObject(TrackedObject* object)
{

}

void SerialSender::RegisterRobot(TrackerCube* tracker)
{
	SerialObjects.push_back(tracker);
}

void SerialSender::SendPacket()
{
	uint32_t ms = (getTickCount() - StartTick) *1000 / getTickFrequency();
	SerialTransmission packet;
	packet.score = 0; // TODO : compute score
	packet.ms = ms;
	packet.PositionPackets.resize(0);
	for (size_t i = 0; i < packet.NumPositions; i++)
	{
		vector<PositionPacket> positions = SerialObjects[i]->ToPacket(i);
		packet.PositionPackets.insert(packet.PositionPackets.end(), positions.begin(), positions.end());
	}
	packet.NumPositions = packet.PositionPackets.size();
	
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
	for (size_t i = 0; i < SerialObjects.size(); i++)
	{
		file << "numeral, X , Y, rot(deg)";
		if (i != SerialObjects.size() -1)
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
	for (size_t i = 0; i < SerialObjects.size(); i++)
	{
		vector<PositionPacket> packets = SerialObjects[i]->ToPacket(i);
		for (size_t j = 0; j < packets.size(); j++)
		{
			file << packets[j].ToCSV();
			if ((i != SerialObjects.size() -1) || (j != packets.size() -1))
			{
				file << ", ";
			}
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