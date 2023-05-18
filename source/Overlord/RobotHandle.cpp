#include "Overlord/RobotHandle.hpp"

#include <sstream>
#include <cassert>
#include <opencv2/core/affine.hpp>
#include <math3d.hpp>
#include <filesystem>
#include "thirdparty/serialib.h"

using namespace Overlord;
using namespace std;
namespace fs = std::filesystem;

#define CheckRunnable if(TimeBudget <= __DBL_EPSILON__ || bridgehandle == nullptr || !bridgehandle->isDeviceOpen()){TimeBudget=0;return 0;}

RobotHandle::RobotHandle()
{
	bridgehandle = nullptr;
	lastTick = chrono::steady_clock::now();
	lastKeepalive = lastTick;
}

RobotHandle::~RobotHandle()
{
	bridgehandle->closeDevice();
	delete bridgehandle;
}

vector<string> RobotHandle::autoDetectTTYACM()
{
	vector<string> serialports;
	string target = "ttyACM";
	for (const auto& entry : fs::directory_iterator("/dev"))
	{
		auto filename = entry.path().filename();
		string filenamestr = filename.string();
		int location = filenamestr.find(target);
		//cout << filename << "@" << location << endl;
		if (location != string::npos)
		{
			serialports.push_back(entry.path().string());
		}
	}
	sort(serialports.begin(), serialports.end());
	return serialports;
}

bool RobotHandle::RegenerateSerial()
{
	if (bridgehandle == nullptr)
	{
		bridgehandle = new serialib();
	}
	bool needregen = false;
	if (!bridgehandle->isDeviceOpen())
	{
		needregen = true;
	}
	std::chrono::duration<double> TimeSinceLastKeepalive = chrono::steady_clock::now() - lastKeepalive;
	if (TimeSinceLastKeepalive.count() > 2)
	{
		needregen = true;
	}
	if (needregen)
	{
		auto serials = autoDetectTTYACM();
		if (serials.size() == 0)
		{
			cout << "No ACM tty detected !" << endl;
			return false;
		}
		lastSerialIndex = lastSerialIndex % serials.size();
		
		string selectedser = serials[lastSerialIndex];
		lastSerialIndex++;
		cout << "No keepalive received, reconnecting uart: " << selectedser <<endl;
		bridgehandle->closeDevice();
		bridgehandle->openDevice(selectedser.c_str(), 115200);
		bridgehandle->clearDTR();
		usleep(1000000);
		bridgehandle->setDTR();
		usleep(1000000);
		lastKeepalive = chrono::steady_clock::now();
	}
	return bridgehandle->isDeviceOpen();
}

void RobotHandle::IndicateMV() 
{
	if (!bridgehandle)
	{
		return;
	}
	if (!bridgehandle->isDeviceOpen())
	{
		return;
	}
	
	
	bridgehandle->writeString("!ledUI2:1\n");
}

void RobotHandle::Tick()
{
	if (!RegenerateSerial())
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
		char requestripcord[] = "!bouton1:\n"; //request ripcord status
		bridgehandle->writeString(requestripcord);
		char keepalivebuffer[256];
		snprintf(keepalivebuffer, sizeof(keepalivebuffer), "!ledUI1:%d\n!I2CSend:%d\n", keepalivestatus, 10+keepalivestatus); 
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
	if (toread > 0)
	{
		numreceived += bridgehandle->readBytes(receivebuffer+numreceived, toread);
	}
	
	
	receivebuffer[numreceived] = '\0';
	if (numreceived > 0)
	{
		//cout << "received : " << receivebuffer << endl;
	}
	int readidx = 0;
	for (int i = 1; i < numreceived; i++)
	{
		if (receivebuffer[i] != '\n')
		{
			continue;
		}
		char* rcvstart = &receivebuffer[readidx];
		readidx = i+1;
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
				if (!IsStarted())
				{
					char golimp[] = "!I2CSend:33\n"; //request ripcord status
					bridgehandle->writeString(golimp);
				}
				
				continue;
			}
		}
		{
			int keepalivenum;
			succ = sscanf(rcvstart, "keepalive:%x", &keepalivenum);
			if (succ == 1)
			{
				cout << "Received keepalive: " << keepalivenum << endl;
				lastKeepalive = chrono::steady_clock::now();
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
	return !RipCordStatus;
}

cv::Affine3d AffineFromPosAndRot(const Vector2dd& pos, const double& rot)
{
	cv::Vec3d offset{pos.x, pos.y, 0};
	Vector2dd forward(rot);
	Vector2dd left(rot+M_PI_2);
	cv::Matx33d rotation(
		forward.x, forward.y, 0, 
		left.x, left.y, 0,
		0,0,1
	);
	cv::Affine3d finalmat(rotation, offset);
	//cout << finalmat.translation() << endl;
	return finalmat;
}

void PosAndRotFromAffine(const cv::Affine3d transform, Vector2dd& pos, double& rot)
{
	cv::Matx33d rotation = transform.rotation();
	rot = GetRotZ(rotation);
	cv::Vec3d translation = transform.translation();
	pos.x = translation[0];
	pos.y = translation[1];
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
	RobotRotationOffset = -rotationRobot+RobotReportedRotation;
	RobotOrigin = RobotReportedPosition - positionRobot.rotate(RobotRotationOffset);

	Vector2dd startpos = {1,0}, endpos = startpos;
	double Startrot = M_PI_4*5/8, endrot = Startrot;
	ToRobotPosition(endpos, endrot);
	ToWorldPosition(endpos, endrot);
	assert((endpos-startpos).length() < PositionTolerance);
	assert(abs(wraptwopi(endrot-Startrot)) < AngleTolerance);

	Vector2dd SanityPos = RobotReportedPosition;
	double SanityRot = RobotReportedRotation;
	ToWorldPosition(SanityPos, SanityRot);
	assert((SanityPos-NewPosition).length() < PositionTolerance);
	assert(abs(wraptwopi(SanityRot-NewRotation)) < AngleTolerance);

	RobotHAL::SetPosition(NewPosition, NewRotation);
}

double RobotHandle::Rotate(double target, double &TimeBudget)
{
	CheckRunnable
	assert(TimeBudget <1);
	char buffer[256];

	Vector2dd pr = {0,0}; double rr = target;
	ToRobotPosition(pr, rr);
	rr = wraptwopi(rr);
	int angledeg = -rr*180.0/M_PI; 
	if (angledeg < 0)
	{
		angledeg += 360;
	}
	bool AtRotation = angledeg == RobotReportedRotation;
	cout << "Sending rotation order : " << angledeg << endl;
	snprintf(buffer, sizeof(buffer), "!I2CSend:31,%d\n", angledeg);
	bridgehandle->writeString(buffer);
	if (!AtRotation)
	{
		TimeBudget = 0;
	}
	return TimeBudget;
}

double RobotHandle::LinearMove(double distance, double &TimeBudget)
{
	CheckRunnable
	assert(TimeBudget <1);
	char buffer[256];
	Vector2dd pr = position + GetForwardVector()*distance; double rr = 0;
	return MoveTo(pr, TimeBudget, distance >= 0 ? ForceDirection::Forward : ForceDirection::Backwards);
}

double RobotHandle::MoveTo(Vector2dd target, double &TimeBudget, ForceDirection direction)
{
	CheckRunnable
	assert(TimeBudget <1);
	char buffer[256];
	Vector2dd pr = target; double rr = 0;
	ToRobotPosition(pr, rr);
	bool backwards;
	switch (direction)
	{
	case ForceDirection::Forward :
		backwards = false;
		break;
	case ForceDirection::Backwards : 
		backwards = true;
		break;
	default:
		backwards = false;
		break;
	}
	Vector2d<int> posmm(pr.x*1000, pr.y*1000);
	bool AtLocation = (pr-RobotReportedPosition).length() < PositionTolerance;
	cout << "Sending position order : " << posmm.ToString() << endl;
	snprintf(buffer, sizeof(buffer), "!I2CSend:30,%d,%d,%d\n", posmm.x, posmm.y, backwards);
	bridgehandle->writeString(buffer);
	if (!AtLocation)
	{
		TimeBudget = 0;
	}
	return TimeBudget; 
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