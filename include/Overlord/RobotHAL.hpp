#pragma once


#include <vector>

namespace Overlord
{

	class LinearMovement
	{
	public:
		double Pos, Speed, Acceleration, Deceleration;
		double MaxSpeed, MinSpeed;
		double TargetPos;

		LinearMovement()
		{};

		LinearMovement(double acceleration, double deceleration, double maxspeed, double minspeed, double pos0=0.0, double target0=0.0, double speed0=0.0)
			:Pos(pos0), Speed(speed0), Acceleration(acceleration), Deceleration(deceleration), MaxSpeed(maxspeed), MinSpeed(minspeed), TargetPos(target0)
		{};

		virtual ~LinearMovement() {};

		constexpr double SpeedDeltaTime(double v0, double v1, double acc)
		{
			return (v1-v0)/acc;
		}

		constexpr double SpeedDeltaDistance(double v0, double v1, double acc)
		{
			double deltaspeed = v1-v0;
			return deltaspeed/acc*(deltaspeed*0.5 + v0);
		};

		enum class MoveABResult
		{
			Done,
			Overshot,
			WrongDirection,
			NoTimeLeft
		};

		double GetBrakingPosition(double v0);
		//Must be going the right direction, will only do a single acceleration/deceleration
		MoveABResult MoveAB(double Target, double& TimeBudget);

		void SetTarget(double NewTarget);

		//Try to move to the target pos for dt maximum time.
		//Returns the time used
		double Tick(double &TimeBudget);
	};


	class RobotHAL
	{
	public:
		double posX, posY, velX, velY;
		LinearMovement PositionLinear;
		LinearMovement ClawHeight, ClawExtension;
		LinearMovement Trays[3];

		RobotHAL(/* args */);
		RobotHAL(RobotHAL* CopyFrom);
		virtual ~RobotHAL();

		virtual double MoveTo(double x, double y, double rot, double &TimeBudget);

		virtual double MoveClaw(double height, double extension);

		virtual double MoveTray(int index, double extension);
	};

} // namespace overlord