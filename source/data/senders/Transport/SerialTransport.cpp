#include "data/senders/Transport/SerialTransport.hpp"

#include <vector>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;

enum class SLIPenc : uint8_t
{
	/// \brief The decimal END character (octal 0300).
	///
	/// Indicates the end of a packet.
	END = 192, 

	/// \brief The decimal ESC character (octal 0333).
	///
	/// Indicates byte stuffing.
	ESC = 219,

	/// \brief The decimal ESC_END character (octal 0334).
	///
	/// ESC ESC_END means END data byte.
	ESC_END = 220,

	/// \brief The decimal ESC_ESC character (ocatal 0335).
	///
	/// ESC ESC_ESC means ESC data byte.
	ESC_ESC = 221
};

vector<string> SerialTransport::autoDetectTTYUSB()
{
	vector<string> USBtities;
	string target = "ttyUSB";
	for (const auto& entry : fs::directory_iterator("/dev"))
	{
		auto filename = entry.path().filename();
		string filenamestr = filename.string();
		int location = filenamestr.find(target);
		//cout << filename << "@" << location << endl;
		if (location != string::npos)
		{
			USBtities.push_back(entry.path().string());
		}
	}
	return USBtities;
}

SerialTransport::SerialTransport(unsigned int BaudRate, bool SelfDetectSerial)
{
	if (!SelfDetectSerial)
	{
		return;
	}
	vector<string> SerialPorts = autoDetectTTYUSB();
	for (size_t i = 0; i < SerialPorts.size(); i++)
	{
		cout << "Serial port found at " << SerialPorts[i] << endl;
	}
		
	Bridge = new serialib();
	if (SerialPorts.size() > 0)
	{
		int success = Bridge->openDevice(SerialPorts[0].c_str(), BaudRate);
		cout << "Result opening serial bridge : " << success << endl;
		if (success != 1)
		{
			cout << "Failed to open the serial bridge, make sure your user is in the dialout group" <<endl;
			cout << "run this ->   sudo usermod -a -G dialout $USER    <- then restart your PC." << endl;
		}
		else
		{
			Connected = true;
		}
		
	}
}

void SerialTransport::Broadcast(const void *buffer, int length)
{
	if (length == 0)
	{
		return;
	}

	size_t read_index  = 0;
	size_t write_index = 0;

	const uint8_t* buffer2 = reinterpret_cast<const uint8_t*>(buffer);

	uint8_t* encodedBuffer = new uint8_t[length*2+2]{0};

	// Double-ENDed, flush any data that may have accumulated due to line 
	// noise.
	encodedBuffer[write_index++] = (uint8_t)SLIPenc::END;

	while (read_index < length)
	{
		if(buffer2[read_index] == (uint8_t)SLIPenc::END)
		{
			encodedBuffer[write_index++] = (uint8_t)SLIPenc::ESC;
			encodedBuffer[write_index++] = (uint8_t)SLIPenc::ESC_END;
			read_index++;
		}
		else if(buffer2[read_index] == (uint8_t)SLIPenc::ESC)
		{
			encodedBuffer[write_index++] = (uint8_t)SLIPenc::ESC;
			encodedBuffer[write_index++] = (uint8_t)SLIPenc::ESC_ESC;
			read_index++;
		}
		else
		{
			encodedBuffer[write_index++] = buffer2[read_index++];
		}
	}
	
	Bridge->writeBytes(encodedBuffer, write_index);
	delete[] encodedBuffer;
}

int SerialTransport::Receive(void *buffer, int maxlength, bool blocking)
{
	return -1;
}