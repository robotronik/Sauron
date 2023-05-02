#include "Overlord/RobotHandle.hpp"

#include <sstream>
#include <cassert>
#include "thirdparty/serialib.h"

using namespace Overlord;
using namespace std;

#define CheckRunnable if(TimeBudget <= __DBL_EPSILON__ || !bridgehandle->isDeviceOpen()){TimeBudget=0;return 0;}

RobotHandle::RobotHandle(serialib* InBridge)
{
	bridgehandle = InBridge;
	lastTick = chrono::steady_clock::now();
}

RobotHandle::~RobotHandle()
{
	delete bridgehandle;
}

void RobotHandle::Tick()
{
	if (!bridgehandle->isDeviceOpen())
	{
		lastTick = chrono::steady_clock::now();
		return;
	}
	auto now = chrono::steady_clock::now();
	std::chrono::duration<double> delta = now - lastTick;

	if (delta.count() > 1.0/50.0)
	{
		char requestpos[] = "!I2CRequest:6\n!I2CSend:20\n"; //request position
		bridgehandle->writeString(requestpos);
		char requestripcord[] = "!Bouton1:\n"; //request ripcord status
		bridgehandle->writeString(requestripcord);
		char keepalivebuffer[256];
		snprintf(keepalivebuffer, sizeof(keepalivebuffer), "!LedUI1:%d\n!I2CSend:%d\n", keepalivestatus, 10+keepalivestatus); 
		//blink led to show communication success on motors and UI
		bridgehandle->writeString(keepalivebuffer);
		keepalivestatus = !keepalivestatus;
		lastTick = chrono::steady_clock::now();
	}
	
	
	
	int sizeleft = sizeof(receivebuffer)-numreceived-1;
	int toread = bridgehandle->available();
	if (sizeleft < toread)
	{
		numreceived = 0;
	}
	numreceived += bridgehandle->readBytes(receivebuffer+numreceived, toread);
	receivebuffer[numreceived] = '\0';

	int readidx = 0;
	for (int i = 1; i < numreceived; i++)
	{
		if (receivebuffer[numreceived] != '\n')
		{
			continue;
		}
		char* rcvstart = &receivebuffer[readidx];
		readidx = numreceived+1;
		int succ;
		{
			int x,y,theta;
			succ = sscanf(rcvstart, "I2C:%d,%d,%d\r\n", &x, &y, &theta);
			if (succ == 3)
			{
				RobotReportedPosition.x = x/1000.0;
				RobotReportedPosition.y = y/1000.0;
				RobotReportedRotation = -theta*M_PI/180.0;
				Vector2dd pw = RobotReportedPosition;
				double rw = RobotReportedRotation;
				ToWorldPosition(pw, rw);
				position = pw;
				Rotation.Pos = rw;
				continue;
			}
		}
		{
			int buttonidx, buttonstatus;
			succ = sscanf(rcvstart, "boutonUI%d:%d", &buttonidx, &buttonstatus);
			if (succ == 2)
			{
				continue;
			}
		}
		{
			int buttonidx, buttonstatus;
			succ = sscanf(rcvstart, "bouton%d:%d", &buttonidx, &buttonstatus);
			if (succ == 2)
			{
				if (buttonidx == 1)
				{
					RipCordStatus = buttonstatus;
				}
				continue;
			}
		}
	}
	
	
	int numleft = numreceived-readidx;
	memmove(receivebuffer, &receivebuffer[readidx], numleft);
	memset(&receivebuffer[numleft], 0, readidx);
	numreceived = numleft;
	
}

bool RobotHandle::HasConnection()
{
	return bridgehandle->isDeviceOpen();
}

bool RobotHandle::IsStarted()
{
	return RipCordStatus;
}

void RobotHandle::ToRobotPosition(Vector2dd& pos, double& rot)
{
	rot += RobotRotationOffset;
	pos = pos.rotate(RobotRotationOffset) + RobotOrigin;
}

void RobotHandle::ToWorldPosition(Vector2dd& pos, double& rot)
{
	pos = (pos-RobotOrigin).rotate(-RobotRotationOffset);
	rot -= RobotRotationOffset;
}

void RobotHandle::SetPosition(Vector2dd NewPosition, double NewRotation)
{
	char posbuffer[256];
	Vector2dd positionRobot = NewPosition;
	double rotationRobot = NewRotation;
	ToRobotPosition(positionRobot, rotationRobot);
	RobotOrigin += positionRobot-RobotReportedPosition;
	RobotRotationOffset += rotationRobot-RobotReportedRotation;
	RobotHAL::SetPosition(NewPosition, NewRotation);
}

double RobotHandle::Rotate(double target, double &TimeBudget)
{
	CheckRunnable
	char buffer[256];

	Vector2dd pr = {0,0}; double rr = target;
	ToRobotPosition(pr, rr);
	rr = LinearMovement::wraptwopi(rr);
	int angledeg = -rr*180.0*M_PI; 
	if (angledeg < 0)
	{
		angledeg += 360;
	}
	if (LastRotate != angledeg)
	{
		snprintf(buffer, sizeof(buffer), "!I2CSend:31,%d\n", angledeg);
		bridgehandle->writeString(buffer);
		LastRotate = angledeg;
	}
	return RobotHAL::Rotate(target, TimeBudget);
}

double RobotHandle::LinearMove(double distance, double &TimeBudget)
{
	CheckRunnable
	char buffer[256];
	Vector2dd pr = position + GetForwardVector()*distance; double rr = 0;
	ToRobotPosition(pr, rr);
	int backwards = distance < 0;
	Vector2d<int> posmm(pr.x*1000, pr.y*1000);
	if (posmm != LastPos)
	{
		snprintf(buffer, sizeof(buffer), "!I2CSend:30,%d,%d,%d\n", posmm.x, posmm.y, backwards);
		bridgehandle->writeString(buffer);
		LastPos = posmm;
	}
	return RobotHAL::LinearMove(distance, TimeBudget);
}

double RobotHandle::MoveClawVertical(double height, double &TimeBudget)
{
	CheckRunnable
	char buffer[256];
	int pos = max(min(height/TrayHeights[3]*750, 750.0), 0.0);
	snprintf(buffer, sizeof(buffer), "!stepper1:%d\n", pos);
	bridgehandle->writeString(buffer);
	return RobotHAL::MoveClawVertical(height, TimeBudget);
}

double RobotHandle::MoveClawExtension(double extension, double &TimeBudget)
{
	CheckRunnable
	char buffer[256];
	int pos = max(min(extension, 1.0), 0.0) * 140;
	snprintf(buffer, sizeof(buffer), "!servo4:%d\n", pos);
	bridgehandle->writeString(buffer);
	return RobotHAL::MoveClawExtension(extension, TimeBudget);
}

double RobotHandle::MoveTray(int index, double extension, double &TimeBudget)
{
	CheckRunnable
	assert(index >= 0 && index < 3);
	char buffer[256];
	int trayidx = index +5;
	int pos = max(min(1.0-extension, 1.0), 0.0) * 180;
	snprintf(buffer, sizeof(buffer), "!servo%d:%d\n", trayidx, pos);
	bridgehandle->writeString(buffer);
	return RobotHAL::MoveTray(index, extension, TimeBudget);
}