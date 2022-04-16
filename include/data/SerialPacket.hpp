#pragma once

#include <vector>
#include <time.h>
#include <stdint.h>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;

struct RobotPacket
{
	uint8_t numero;
	float X;
	float Y;
	float rotation; //-pi to pi

	String ToCSV();
};

struct PaletPacket //palets sur l'arène uniquement
{
	uint8_t TagID;
	float X;
	float Y;
	float rotation; //-pi to pi
};

struct SerialPacketOut
{
public:
	uint8_t NumRobots;
	uint8_t NumPalets;
	uint16_t score; //score visuel, inclue uniquement les palets
	uint32_t ms; //enough for 49 days
	vector<RobotPacket> robots;
	vector<PaletPacket> palets;
private:
	char* buffer;

public:
	SerialPacketOut();
	~SerialPacketOut();

	uint32_t GetSizeNoVector() const;
	uint32_t GetPacketSize() const;
	void* ToBuffer();
	bool FromBuffer(void* buffer);
};
