#include "Overlord/RobotHAL.hpp"

#include <cmath>
#include <cassert>
#include <cstring>
#include <limits>
#include <iostream>


using namespace std;

void Overlord::LinearMovement::SetTarget(double NewTarget)
{
	TargetPos = NewTarget;
}

double Overlord::LinearMovement::GetBrakingPosition(double v0) 
{
	double v1 = copysign(MinSpeed, v0);
	double dec = -copysign(Deceleration, v0);
	return SpeedDeltaDistance(v0, v1, dec);
}

Overlord::LinearMovement::MoveABResult Overlord::LinearMovement::MoveAB(double Target, double &TimeBudget)
{
	double dx = Target - Pos;
	bool InitialMoving = abs(Speed) > MinSpeed;
	bool GoingTheRightWay = Speed*dx>0;
	if (InitialMoving && !GoingTheRightWay)
	{
		cout << "Going the wrong way" << endl;
		return MoveABResult::WrongDirection;
	}
	
	double TimeToMaxSpeed, TimeFromMaxSpeed;
	double DistToMaxSpeed, DistFromMaxSpeed;
	Speed = abs(Speed) < MinSpeed ? copysign(MinSpeed, dx) : Speed; //Start speed
	double acc = copysign(Acceleration, dx);
	double dec = -copysign(Deceleration, dx);
	{ //current speed to full speed
		double v0 = Speed;
		double v1 = copysign(MaxSpeed, dx);
		
		TimeToMaxSpeed = SpeedDeltaTime(v0, v1, acc);
		DistToMaxSpeed = SpeedDeltaDistance(v0, v1, acc);
		assert(TimeToMaxSpeed >-__DBL_EPSILON__);
	}
	{ //full speed to min speed
		double v0 = copysign(MaxSpeed, dx);
		double v1 = copysign(MinSpeed, dx);
		TimeFromMaxSpeed = SpeedDeltaTime(v0, v1, dec);
		DistFromMaxSpeed = SpeedDeltaDistance(v0, v1, dec);
		assert(TimeFromMaxSpeed >0);
	}
	double BrakingDistance = GetBrakingPosition(Speed);
	cout 
	<< "dx = " << dx 
	<< " | Speed = " << Speed 
	<< " | Acc = " << acc 
	<< " | Dec = " << dec 
	<< " | Time to max speed = " << TimeToMaxSpeed 
	<< " | Dist to max speed = " << DistToMaxSpeed 
	<< " | Time from max speed = " << TimeFromMaxSpeed 
	<< " | Dist from max speed = " << DistFromMaxSpeed
	<< " | Braking distance = " << BrakingDistance
	<< endl;
	if (abs(BrakingDistance) > abs(dx)) //will overshoot, brake hard
	{
		cout << "Will overshoot" << endl;
		double v0 = Speed;
		double v1 = copysign(MinSpeed, Speed);
		double TimeToStop = SpeedDeltaTime(v0, v1, dec);
		if (TimeBudget < TimeToStop)
		{
			double v1 = Speed + TimeBudget*dec;
			Pos += SpeedDeltaDistance(Speed, v1, dec);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		TimeBudget-=TimeToStop;
	}
	else if (abs(DistFromMaxSpeed + DistToMaxSpeed) > abs(dx)) //no time for full speed
	{
		cout << "Speed triangle" << endl;
		double step = copysign(MaxSpeed, Speed) - Speed;
		step /= 2;
		double peakspeed = Speed + step;
		step /= 2;
		double TimeToPeak, TimeFromPeak;
		double DistToPeak, DistFromPeak;
		for (int i = 0; i < 100; i++)
		{
			double v0 = Speed;
			double v1 = peakspeed;
			double v2 = copysign(MinSpeed, dx);
			TimeToPeak = SpeedDeltaTime(v0, v1, acc);
			DistToPeak = SpeedDeltaDistance(v0, v1, acc);
			TimeFromPeak = SpeedDeltaTime(v1, v2, dec);
			DistFromPeak = SpeedDeltaDistance(v1, v2, dec);
			assert(TimeToPeak >-__DBL_EPSILON__);
			assert(TimeFromPeak >-__DBL_EPSILON__);
			if (abs(DistToPeak+DistFromPeak) > abs(dx))
			{
				//peak speed is lower
				peakspeed -= step;
			}
			else
			{
				peakspeed += step;
			}
			step /= 2;
		}
		if (TimeBudget < TimeToPeak) //Not enough time to reach peak
		{
			double v1 = Speed + TimeBudget*acc;
			Pos += SpeedDeltaDistance(Speed, v1, acc);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		Pos += DistToPeak;
		TimeBudget -= TimeToPeak;
		Speed = peakspeed;
		if (TimeBudget < TimeFromPeak) //Not enough time to fully stop
		{
			double v1 = Speed + TimeBudget*dec;
			Pos += SpeedDeltaDistance(Speed, v1, dec);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		TimeBudget -= TimeFromPeak;
	}
	else
	{
		cout << "Constant speed" << endl;
		double DistFullSpeed = copysign(abs(dx) - abs(DistToMaxSpeed + DistFromMaxSpeed), dx);
		double TimeFullSpeed = DistFullSpeed/copysign(MaxSpeed, dx);
		assert(TimeFullSpeed > -__DBL_EPSILON__);

		if (TimeBudget < TimeToMaxSpeed)
		{
			double v1 = Speed + TimeBudget*acc;
			Pos += SpeedDeltaDistance(Speed, v1, acc);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		Pos += DistToMaxSpeed;
		TimeBudget -= TimeToMaxSpeed;
		Speed = copysign(MaxSpeed, dx);
		if (TimeBudget < TimeFullSpeed)
		{
			Pos += Speed * TimeBudget;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		Pos += DistFullSpeed;
		TimeBudget -= TimeFullSpeed;
		if (TimeBudget < TimeFromMaxSpeed)
		{
			double v1 = Speed + TimeBudget*dec;
			Pos += SpeedDeltaDistance(Speed, v1, dec);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		TimeBudget -= TimeFromMaxSpeed;
	}
	Pos = Target;
	Speed = 0;
	return MoveABResult::Done;
	
}

double Overlord::LinearMovement::Tick(double &TimeBudget)
{
	MoveABResult result;
	do
	{
		result = MoveAB(TargetPos, TimeBudget);
		switch (result)
		{
		case MoveABResult::WrongDirection :
			double inttarget = Pos + GetBrakingPosition(Speed);
			cout << "Wrong direction : new target is " << inttarget << endl;
			result = MoveAB(inttarget, TimeBudget);
			break;
		}
	} while (Pos != TargetPos && TimeBudget > __DBL_EPSILON__);
	return TimeBudget;
}

Overlord::RobotHAL::RobotHAL()
{
}

Overlord::RobotHAL::RobotHAL(RobotHAL* CopyFrom)
{
	memcpy(this, CopyFrom, sizeof(RobotHAL));
}

Overlord::RobotHAL::~RobotHAL()
{
}

double Overlord::RobotHAL::MoveTo(double x, double y, double rot, double &TimeBudget)
{
	while (TimeBudget > 0)
	{
		double dx = x-posX;
		double dy = y-posY;
		double dmag = sqrt(dx*dx+dy*dy);
		if (dmag < 0.001)
		{
			break;
		}
		
		double velmag = sqrt(velX*velX + velY*velY);
		double dp = dx*velX+dy*velY;
		double costheta = dp/(dmag*velmag);
		if (costheta/velmag > 900) //needs to be going the same direction the faster it's going
		{
			PositionLinear.Pos = 0;
			PositionLinear.Speed = velmag;
			PositionLinear.SetTarget(dmag);
			TimeBudget -= PositionLinear.Tick(TimeBudget);
			posX += PositionLinear.Pos*dx/dmag;
			posY += PositionLinear.Pos*dy/dmag;
		}
		else
		{
			PositionLinear.Pos = 0;
			PositionLinear.Speed = velmag;
			PositionLinear.SetTarget(PositionLinear.GetBrakingPosition(velmag));
			TimeBudget -= PositionLinear.Tick(TimeBudget);
			posX += PositionLinear.Pos*velX/velmag;
			posY += PositionLinear.Pos*velY/velmag;
		}
	}
	return TimeBudget;
}

double Overlord::RobotHAL::MoveClaw(double height, double extension)
{
	return 0;
}

double Overlord::RobotHAL::MoveTray(int index, double extension)
{
	return 0;
}
