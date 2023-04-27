#include "Overlord/RobotHandle.hpp"

#include <sstream>
#include "thirdparty/serialib.h"

using namespace Overlord;
using namespace std;

RobotHandle::RobotHandle(serialib* InBridge)
{
	bridgehandle = InBridge;
}

RobotHandle::~RobotHandle()
{
	delete bridgehandle;
}

void RobotHandle::Tick()
{
	if (!bridgehandle->isDeviceOpen())
	{
		return;
	}
	
	char requestpos[] = "!I2CRequest:6\n!I2CSend:20\n";
	bridgehandle->writeString(requestpos);
	int sizeleft = sizeof(receivebuffer)-numreceived;
	int toread = bridgehandle->available();
	if (sizeleft < toread)
	{
		numreceived = 0;
	}
	numreceived += bridgehandle->readBytes(receivebuffer+numreceived, toread);
	//find the last ! with a \n following it
	int lastbang = -1, lastnewline = -1, previousbang = -1, previousnewline = 0;
	for (int i = 0; i < numreceived; i++)
	{
		switch (receivebuffer[i])
		{
		case '!':
			lastbang = i;
			break;
		case '\n':
			previousnewline = lastnewline;
			lastnewline = i;
			previousbang = lastbang;
		default:
			break;
		}
	}
	if (lastnewline == -1)
	{
		return;
	}
	char* rcvstart = &receivebuffer[previousnewline];
	int x,y,theta;
	int succ = sscanf(rcvstart, "I2C:%d,%d,%d\n", &x, &y, &theta);
	if (succ != 3)
	{
		return;
	}
	int numleft = numreceived-lastnewline+1;
	memmove(receivebuffer, &receivebuffer[lastnewline+1], numleft);
	memset(&receivebuffer[numleft], 0, lastnewline+1);
	numreceived = numleft;
	position.x = x/1000.0;
	position.y = y/1000.0;
	Rotation.Pos = -theta*M_PI/180.0;
}

double RobotHandle::Rotate(double target, double &TimeBudget)
{
	if (TimeBudget <= __DBL_EPSILON__)
	{
		return 0;
	}
	if (!bridgehandle->isDeviceOpen())
	{
		TimeBudget = 0;
		return 0;
	}
	char buffer[256];
	int angledeg = -target*180.0*M_PI; 
	if (angledeg < 0)
	{
		angledeg += 360;
	}
	
	snprintf(buffer, sizeof(buffer), "!I2CSend:31,%d\n", angledeg);
	bridgehandle->writeString(buffer);
	return RobotHAL::Rotate(target, TimeBudget);
}

double RobotHandle::LinearMove(double distance, double &TimeBudget)
{
	if (TimeBudget <= __DBL_EPSILON__)
	{
		return 0;
	}
	if (!bridgehandle->isDeviceOpen())
	{
		TimeBudget = 0;
		return 0;
	}
	char buffer[256];
	auto target = position + GetForwardVector()*distance;
	int backwards = distance < 0;
	int xt = target.x*1000;
	int yt = target.y*1000;
	
	snprintf(buffer, sizeof(buffer), "!I2CSend:30,%d,%d,%d\n", xt, yt, backwards);
	bridgehandle->writeString(buffer);
	return RobotHAL::LinearMove(distance, TimeBudget);
}