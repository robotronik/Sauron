#include "Overlord/RobotHAL.hpp"

#include <cmath>
#include <cassert>
#include <cstring>
#include <limits>
#include <iostream>


using namespace std;
using namespace Overlord;

double LinearMovement::wraptwopi(double in)
{
	double rem = fmod(in, M_PI*2);
	if (rem < -M_PI)
	{
		return rem + M_PI*2;
	}
	else if (rem > M_PI)
	{
		return rem - M_PI*2;
	}
	return rem;
}

double LinearMovement::closestangle(double x, double ref)
{
	double delta = wraptwopi(x-ref);
	return delta + ref;
}

void LinearMovement::SetTarget(double NewTarget)
{
	TargetPos = NewTarget;
}

void LinearMovement::SetTargetAngular(double angle)
{
	Pos = wraptwopi(Pos);
	TargetPos = closestangle(angle, Pos);
}

double LinearMovement::GetBrakingDistance(double v0) 
{
	double v1 = copysign(MinSpeed, v0);
	double dec = -copysign(Deceleration, v0);
	return SpeedDeltaDistance(v0, v1, dec);
}

LinearMovement::MoveABResult LinearMovement::MoveAB(double Target, double &TimeBudget)
{
	double dx = Target - Pos;
	bool InitialMoving = abs(Speed) > MinSpeed + __DBL_EPSILON__;
	bool GoingTheRightWay = Speed*dx>0;
	if (InitialMoving && !GoingTheRightWay)
	{
		//cout << "Going the wrong way" << endl;
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
		//assert(TimeToMaxSpeed >-__DBL_EPSILON__);
	}
	{ //full speed to min speed
		double v0 = copysign(MaxSpeed, dx);
		double v1 = copysign(MinSpeed, dx);
		TimeFromMaxSpeed = SpeedDeltaTime(v0, v1, dec);
		DistFromMaxSpeed = SpeedDeltaDistance(v0, v1, dec);
		assert(TimeFromMaxSpeed >0);
	}
	double BrakingDistance = GetBrakingDistance(Speed);
	/*cout 
	<< "dx = " << dx 
	<< " | Speed = " << Speed 
	<< " | Acc = " << acc 
	<< " | Dec = " << dec 
	<< " | Time to max speed = " << TimeToMaxSpeed 
	<< " | Dist to max speed = " << DistToMaxSpeed 
	<< " | Time from max speed = " << TimeFromMaxSpeed 
	<< " | Dist from max speed = " << DistFromMaxSpeed
	<< " | Braking distance = " << BrakingDistance
	<< endl;*/
	if (abs(BrakingDistance) > abs(dx)) //will overshoot, brake hard
	{
		//cout << "Will overshoot" << endl;
		double v0 = Speed;
		double v1 = copysign(MinSpeed, dx);
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
		//cout << "Speed triangle" << endl;
		double step = copysign(MaxSpeed, dx) - Speed;
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
		assert(TimeToPeak >-__DBL_EPSILON__);
		assert(TimeFromPeak >-__DBL_EPSILON__);
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
		//cout << "Constant speed" << endl;
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

double LinearMovement::Tick(double &TimeBudget)
{
	MoveABResult result;
	int i = 0;
	do
	{
		result = MoveAB(TargetPos, TimeBudget);
		switch (result)
		{
		case MoveABResult::WrongDirection :
			double inttarget = Pos + GetBrakingDistance(Speed);
			//cout << "Wrong direction : new target is " << inttarget << endl;
			result = MoveAB(inttarget, TimeBudget);
			break;
		}
		i++;
	} while (abs(Pos - TargetPos)>__DBL_EPSILON__ && TimeBudget > __DBL_EPSILON__ && i < 10);
	return TimeBudget;
}

RobotHAL::RobotHAL()
{
}

RobotHAL::~RobotHAL()
{
}

bool RobotHAL::IsLocationValid(Vector2d<double> pos, double rot) const
{
	return false; //todo
}

Vector2d<double> RobotHAL::GetForwardVector()
{
	return Vector2d<double>(Rotation.Pos);
}

Vector2d<double> RobotHAL::GetStoppingPosition()
{
	double velmag = PositionLinear.Speed;
	double stoppingpos1d = PositionLinear.GetBrakingDistance(PositionLinear.Speed);
	if (velmag <= PositionLinear.MinSpeed + __DBL_EPSILON__)
	{
		return position;
	}
	return position + GetForwardVector()*stoppingpos1d;
}

double RobotHAL::Rotate(double target, double &TimeBudget)
{
	if (TimeBudget < __DBL_EPSILON__)
	{
		return TimeBudget;
	}
	Rotation.SetTargetAngular(target);
	Rotation.Tick(TimeBudget);
	return TimeBudget;
}

double RobotHAL::LinearMove(double distance, double &TimeBudget)
{
	if (TimeBudget < __DBL_EPSILON__)
	{
		return TimeBudget;
	}
	PositionLinear.SetTarget(distance);
	PositionLinear.Pos = 0;
	PositionLinear.Tick(TimeBudget);
	double xv, yv;
	auto forward = GetForwardVector();
	position += forward * PositionLinear.Pos;
	return TimeBudget;
}

double RobotHAL::MoveTo(Vector2d<double> target, double &TimeBudget, ForceDirection direction)
{
	Vector2d<double> deltapos;
	double dist;
	double angleneeded;
	double dangle;
	bool goingbackwards;
	auto reevaluate = [&](){
	deltapos = target-position;
	dist = deltapos.length();
	angleneeded = deltapos.angle();
	dangle = LinearMovement::wraptwopi(angleneeded-Rotation.Pos);
	switch (direction)
	{
	case ForceDirection::Backwards :
		goingbackwards = true;
		break;
	case ForceDirection::Forward : 
		goingbackwards = false;
		break;
	default:
		goingbackwards = abs(dangle) > M_PI_2;
		break;
	}
	};

	reevaluate();
	
	if (goingbackwards)
	{
		angleneeded = LinearMovement::wraptwopi(angleneeded + M_PI);
		dangle = LinearMovement::wraptwopi(dangle + M_PI);
	}
	if (abs(dangle)> 1/180.0*M_PI) //not going the right way
	{
		if (abs(PositionLinear.Speed) > PositionLinear.MinSpeed + __DBL_EPSILON__) //moving
		{
			LinearMove(PositionLinear.GetBrakingDistance(PositionLinear.Speed), TimeBudget); //stop
			reevaluate();
		}
	}
	Rotate(angleneeded, TimeBudget);
	LinearMove(goingbackwards ? -dist : dist, TimeBudget);
	return TimeBudget;
}

double RobotHAL::MoveTo(Vector2d<double> target, double rot, double &TimeBudget, ForceDirection direction)
{
	MoveTo(target, TimeBudget, direction);
	double drot = LinearMovement::wraptwopi(rot-Rotation.Pos);
	if (abs(drot)> 10.0/180.0*M_PI && TimeBudget > __DBL_EPSILON__) //needs to rotate
	{
		Rotate(drot, TimeBudget);
	}
	return TimeBudget;
}

double RobotHAL::MoveToOffset(Vector2d<double> target, Vector2d<double> offset, double &TimeBudget)
{
	if ((position + offset.rotate(Rotation.Pos) - target).length() < 0.001) //at target
	{
		return TimeBudget;
	}
	cout << "new attempt" <<endl;
	
	auto deltapos = target-position;
	if (deltapos.lengthsquared() < offset.lengthsquared())
	{
		double dneed = offset.length() - deltapos.length() + __DBL_EPSILON__;
		cout << "singularity : moving " << dneed << endl;
		double fmove = offset.x >= 0 ? -dneed : dneed; //if the offset is in the front, move back, else move forward
		MoveTo(position + GetForwardVector() * fmove, TimeBudget, offset.x >= 0 ? ForceDirection::Backwards : ForceDirection::Forward);
		if (TimeBudget < __DBL_EPSILON__)
		{
			return 0;
		}
		deltapos = target-position;
	}
	double angle = deltapos.angle();
	double dplen = deltapos.lengthsquared();
	double xsq = dplen - offset.y*offset.y;
	assert(xsq>=0);
	double x = sqrt(xsq);
	double offsetangle = atan2(offset.y, x);
	
	
	vector<double> anglestotry = {angle-offsetangle};
	vector<double> distancetotry = {x-offset.x};
	int correctangle = -1, correctdistance = -1;
	for (int i = 0; i < anglestotry.size(); i++)
	{
		for (int j = 0; j < distancetotry.size(); j++)
		{
			double thisangle = LinearMovement::wraptwopi(anglestotry[i]);
			Vector2d<double> fp = position + Vector2d<double>(thisangle) * distancetotry[j]; //final position of the robot : go forward the distance
			Vector2d<double> ofp = fp + offset.rotate(thisangle); //final pos of the offset
			auto error = ofp-target;
			auto errorloc = error.rotate(-thisangle);
			if (error.length() < 0.001)
			{
				cout << "\tCorrect is " << i << " and " << j << ", angle is " << thisangle*180/M_PI <<endl;
				correctangle = i;
				correctdistance = j;
				goto foundcorrect;
			}
			else
			{
				cout << "\tIncorrect " << i << " and " << j << " : (" << errorloc.x << ", " << errorloc.y << ")" <<endl;
			}
			
		}
	}
foundcorrect:
	assert(correctangle != -1);
	
	
	auto offsettarget = position + Vector2d<double>(anglestotry[correctangle]) * distancetotry[correctdistance];
	MoveTo(offsettarget, TimeBudget, offset.x >= 0 ? ForceDirection::Forward : ForceDirection::Backwards);
	return TimeBudget;
}

double RobotHAL::MoveClaw(double height, double extension, double& TimeBudget)
{
	return 0;
}

double RobotHAL::MoveTray(int index, double extension, double& TimeBudget)
{
	return 0;
}
