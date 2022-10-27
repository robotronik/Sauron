#include "data/senders/SerialSender.hpp"

#include "math3d.hpp"
#include "filesystem"

#include "data/SerialPacket.hpp"

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

SerialSender::SerialSender(bool SelfDetectSerial)
{
	if (!SelfDetectSerial)
	{
		return;
	}
	vector<String> SerialPorts = SerialSender::autoDetectTTYUSB();
	for (size_t i = 0; i < SerialPorts.size(); i++)
	{
		cout << "Serial port found at " << SerialPorts[i] << endl;
	}
		
	serialib* bridge = new serialib();
	if (SerialPorts.size() > 0)
	{
		int success = bridge->openDevice(SerialPorts[0].c_str(), SerialTransmission::BaudRate);
		cout << "Result opening serial bridge : " << success << endl;
		if (success != 1)
		{
			cout << "Failed to open the serial bridge, make sure your user is in the dialout group" <<endl;
			cout << "run this ->   sudo usermod -a -G dialout $USER    <- then restart your PC." << endl;
		}
	}
	Bridge = bridge;
}

SerialSender::SerialSender(serialib* InBridge)
{
	Bridge = InBridge;
}

SerialSender::~SerialSender()
{
	delete Bridge;
}

serialib* SerialSender::GetBridge()
{
	return Bridge;
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
		vector<PositionPacket> positions = RegisteredObjects[i]->ToPacket(i);
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