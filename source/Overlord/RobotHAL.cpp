#include "Overlord/RobotHAL.hpp"

#include <cmath>
#include <cassert>
#include <cstring>
#include <limits>
#include <iostream>


using namespace std;
using namespace Overlord;

#define ReturnIfNoBudget {if(TimeBudget < __DBL_EPSILON__) {return 0;}}

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

double LinearMovement::GetStoppingPosition()
{
	return Pos+GetBrakingDistance(Speed);
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
		double step = copysign(MaxSpeed, dx) - copysign(MinSpeed, dx);
		step /= 2;
		double peakspeed = copysign(MinSpeed, dx) + step;
		step /= 2;
		double TimeToPeak, TimeFromPeak;
		double DistToPeak, DistFromPeak;
		for (; abs(step) > __DBL_EPSILON__;)
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
		//assert(TimeToPeak >-__DBL_EPSILON__*8);
		//assert(TimeFromPeak >-__DBL_EPSILON__*8);
		TimeToPeak = max(TimeToPeak, 0.0);
		TimeFromPeak = max(TimeFromPeak, 0.0);
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

bool RobotHAL::IsLocationValid(Vector2dd pos, double rot) const
{
	return false; //todo
}

Vector2dd RobotHAL::GetForwardVector()
{
	return Vector2dd(Rotation.Pos);
}

Vector2dd RobotHAL::GetStoppingPosition()
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
	ReturnIfNoBudget
	Rotation.SetTargetAngular(target);
	Rotation.Tick(TimeBudget);
	return TimeBudget;
}

double RobotHAL::LinearMove(double distance, double &TimeBudget)
{
	ReturnIfNoBudget
	PositionLinear.SetTarget(distance);
	PositionLinear.Pos = 0;
	PositionLinear.Tick(TimeBudget);
	double xv, yv;
	auto forward = GetForwardVector();
	position += forward * PositionLinear.Pos;
	return TimeBudget;
}

double RobotHAL::MoveTo(Vector2dd target, double &TimeBudget, ForceDirection direction)
{
	assert(target.abs() < Vector2dd(1.5,1.0));
	Vector2dd deltapos;
	double dist;
	double angleneeded;
	double dangle;
	bool goingbackwards;
	auto reevaluate = [&](){
		deltapos = target-position;
		dist = deltapos.length();
		angleneeded = deltapos.angle();
		dangle = wraptwopi(angleneeded-Rotation.Pos);
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
			//cout << "MoveTo : detected " << (goingbackwards ? "backwards" : "forward") << " move" << endl;
			break;
		}
		if (goingbackwards)
		{
			angleneeded = wraptwopi(angleneeded + M_PI);
			dangle = wraptwopi(dangle + M_PI);
		}
	};

	reevaluate();
	
	
	if (abs(dangle)>AngleTolerance) //not going the right way
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

bool RobotHAL::IsOffsetInSingularity(Vector2dd target, Vector2dd offset)
{
	auto deltapos = target-position;
	return deltapos.lengthsquared() < offset.lengthsquared() + __DBL_EPSILON__;
}

vector<pair<double, double>> RobotHAL::GetOffsetTarget(Vector2dd target, Vector2dd offset)
{
	auto deltapos = target-position;
	double angle = deltapos.angle();
	double dplen2 = deltapos.lengthsquared();
	double xsq = dplen2 - offset.y*offset.y;
	assert(xsq>=0);
	double x = sqrt(xsq);
	double offsetangle = asin(offset.y/sqrt(dplen2));
	
	
	vector<double> anglestotry = {angle-offsetangle, angle+offsetangle, angle-offsetangle+M_PI, angle+offsetangle+M_PI};
	vector<pair<double, double>> solutions;
	int correctangle = -1;
	//goto foundcorrect;
	for (int i = 0; i < anglestotry.size(); i++)
	{
		double thisangle = wraptwopi(anglestotry[i]); //thisangle will be the rotation we take
		Vector2dd offsetrotworld = position + offset.rotate(thisangle); //final pos of the offset
		Vector2dd deltalin = target-offsetrotworld; //final position of the robot : go forward the distance
		auto forward = Vector2dd(thisangle);
		auto side = Vector2d(thisangle+M_PI_2);
		double dist = deltalin.dot(forward);
		auto sideerror = deltalin.dot(side);
		if (abs(sideerror) < 1e-5)
		{
			solutions.push_back({dist, thisangle});
		}
	}
	return solutions;
}

double RobotHAL::MoveToOffset(Vector2dd target, Vector2dd offset, double &TimeBudget, ForceDirection direction)
{
	if ((position + offset.rotate(Rotation.Pos) - target).length() < PositionTolerance) //at target
	{
		return TimeBudget;
	}
	//cout << "new attempt" <<endl;
	
	auto deltapos = target-position;
	if (IsOffsetInSingularity(target, offset))
	{
		auto targetlocal = deltapos.rotate(-Rotation.Pos);
		auto deltaloc = offset + targetlocal;
		double dneed = deltaloc.length();
		//cout << "target is nearer to robot than offset " << deltaloc.ToString() << " : moving " << dneed << endl;
		double fmove = deltaloc.x > 0 ? -dneed : dneed; //if the offset is in the front, move back, else move forward
		MoveTo(position + GetForwardVector() * fmove, TimeBudget/*, offset.x >= 0 ? ForceDirection::Backwards : ForceDirection::Forward*/);
		ReturnIfNoBudget
		deltapos = target-position;
	}
	pair<double, double> bestsolution= {0,0};
	ForceDirection memdir;
	auto reevaluate = [&](){
		auto allsolutions = GetOffsetTarget(target, offset);
		//cout << "Found " << allsolutions.size() << " solutions to go to the target with offset" <<endl;
		memdir = direction;
		pair<double, double> forwardsolution = {0,0}, backwardssolution = {0,0};
		bool HasForward = false, HasBackwards = false;
		for (int i = 0; i < allsolutions.size(); i++)
		{
			auto& thissolution = allsolutions[i];
			auto solutionforward = Vector2dd(thissolution.second);
			bool IsForwardSolution = thissolution.first > 0;
			if (IsForwardSolution && (!HasForward || abs(thissolution.first)<abs(forwardsolution.first)))
			{
				forwardsolution = thissolution;
				HasForward= true;
			}
			else if (!IsForwardSolution && (!HasBackwards || abs(thissolution.first)<abs(backwardssolution.first)))
			{
				backwardssolution = thissolution;
				HasBackwards= true;
			}
		}
		
		if (!HasBackwards) //must go forward
		{
			memdir = ForceDirection::Forward;
		}
		else if (!HasForward) //must go backwards
		{
			memdir = ForceDirection::Backwards;
		}
		else if (direction == ForceDirection::None) //choose what's best
		{
			if (abs(forwardsolution.first) < abs(backwardssolution.first))
			{
				memdir = ForceDirection::Forward;
			}
			else
			{
				memdir = ForceDirection::Backwards;
			}
		}
		else //user-decided
		{
			memdir = direction;
		}
		
		
		switch (memdir)
		{
		case ForceDirection::Forward :
			assert(HasForward);
			bestsolution = forwardsolution;
			//cout << "Forward";
			break;
		case ForceDirection::Backwards :
			assert(HasBackwards);
			bestsolution = backwardssolution;
			//cout << "Backwards";
			break;
		}
		//cout << " solution : linear " << bestsolution.first << " angle " << bestsolution.second*180.0/M_PI << endl;
	};
	
	reevaluate();
	
	MoveTo(position+Vector2dd(bestsolution.second) * bestsolution.first, TimeBudget, memdir);
	ReturnIfNoBudget
	reevaluate();
	Rotate(bestsolution.second, TimeBudget);
	ReturnIfNoBudget
	
	//cout << "Delta after offset move : " << (position-offsettarget).ToString() 
	//<< " rotation : " << wraptwopi(anglestotry[correctangle]-Rotation.Pos)*180.0/M_PI << endl;

	auto offworld = GetOffsetWorld(offset);
	double distancetotarget = (offworld-target).length();
	//assert(distancetotarget < PositionTolerance);
	return TimeBudget;
}

double RobotHAL::MovePath(Vector2dd Target, double& TimeBudget, ForceDirection direction)
{
	auto path = pf->Pathfind(position, Target);
	if (!path.has_value())
	{
		TimeBudget = 0;
		return 0;
	}
	for (int i = 1; i < path.value().size(); i++)
	{
		ReturnIfNoBudget
		MoveTo(path.value()[i], TimeBudget, direction);
	}
	return TimeBudget;
}

double RobotHAL::MovePathOffset(Vector2dd Target, Vector2dd offset, double& TimeBudget)
{
	Vector2dd dir = (Target-position).normalized();
	Vector2dd targetmod = Target - dir*offset.length();
	auto path = pf->PathfindOffset(position, Target, offset);
	if (!path.has_value())
	{
		TimeBudget = 0;
		return 0;
	}
	Path& p  = path.value();
	for (int i = 1; i < p.size(); i++)
	{
		ReturnIfNoBudget
		ForceDirection dir = ForceDirection::None;
		if (i == p.size()-1)
		{
			double forwardangle = (p[i]-position).angle();
			auto forwardendpos = offset.rotate(forwardangle)+p[i];
			double forwarddisttotarget = (Target-forwardendpos).length();
			if (forwarddisttotarget > offset.length())
			{
				dir = ForceDirection::Backwards;
			}
			else
			{
				dir = ForceDirection::Forward;
			}
		}
		
		MoveTo(p[i], TimeBudget, dir);
	}
	return TimeBudget;
}

double RobotHAL::MoveClawVertical(double height, double& TimeBudget)
{
	ReturnIfNoBudget
	ClawHeight.SetTarget(height);
	ClawHeight.Tick(TimeBudget);
	return TimeBudget;
}

double RobotHAL::MoveClawExtension(double extension, double &TimeBudget)
{
	ReturnIfNoBudget
	ClawExtension.SetTarget(extension);
	ClawExtension.Tick(TimeBudget);
	return TimeBudget;
}

double RobotHAL::MoveTray(int index, double extension, double& TimeBudget)
{
	ReturnIfNoBudget
	assert(index < sizeof(Trays)/sizeof(Trays[0]));
	Trays[index].SetTarget(extension);
	Trays[index].Tick(TimeBudget);
	return TimeBudget;
}

double RobotHAL::MoveClawTo(double height, double &TimeBudget, double InitialExtension, double FinalExtension)
{
	//if not at the target, close the claws and retract the trays
	if (abs(ClawHeight.Pos-height) > PositionTolerance)
	{
		MoveClawExtension(InitialExtension, TimeBudget);
		double minbudget = TimeBudget;
		/*for (int i = 0; i < 3; i++) //retract all trays at once
		{
			double localbudget = TimeBudget;
			MoveTray(i, 0, localbudget);
			if (localbudget < minbudget)
			{
				minbudget = localbudget;
			}
		}*/
		TimeBudget = minbudget;
		MoveClawVertical(height, TimeBudget);
	}
	MoveClawExtension(FinalExtension, TimeBudget);
	return TimeBudget;
}
