#pragma once


#include "Overlord/Collision/Vector2d.hpp"
#include "Overlord/Collision/Pathfinder.hpp"
#include <vector>
#include <utility>
#include <optional>
#include <memory>

namespace Overlord
{

	class LinearMovement
	{
	public:
		double Pos=0, Speed=0, Acceleration=1, Deceleration=2;
		double MaxSpeed=1, MinSpeed=0;
		double TargetPos;

		LinearMovement()
		{};

		LinearMovement(double acceleration, double deceleration, double maxspeed, double minspeed, double pos0=0.0, double target0=0.0, double speed0=0.0)
			:Pos(pos0), Speed(speed0), Acceleration(acceleration), Deceleration(deceleration), MaxSpeed(maxspeed), MinSpeed(minspeed), TargetPos(target0)
		{};

		virtual ~LinearMovement() {};

		static double closestangle(double x, double ref);

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

		double GetBrakingDistance(double v0);

		double GetStoppingPosition();
		//Must be going the right direction, will only do a single acceleration/deceleration
		MoveABResult MoveAB(double Target, double& TimeBudget);

		void SetTarget(double NewTarget);

		void SetTargetAngular(double angle);

		//Try to move to the target pos for dt maximum time.
		//Returns the time used
		double Tick(double &TimeBudget);
	};


	class RobotHAL
	{
	public:
		enum class ForceDirection
		{
			None,
			Forward,
			Backwards
		};

		std::vector<double> TrayHeights = {0,0.04,0.16,0.25};
		Vector2dd ClawPickupPosition = {0.18,0};
		Vector2dd CherryPickupPosition = {-0.11,-0.155};
		Vector2dd CherryDepositPosition = {-0.11, -0.09};
		Vector2dd position;
		LinearMovement PositionLinear;
		LinearMovement Rotation;
		LinearMovement ClawHeight, ClawExtension;
		LinearMovement Trays[3];
		std::shared_ptr<Pathfinder> pf;
		bool BlueTeam = true;

		Vector2dd RobotExtent = {0.21, 0.31};
		double PathfindingRadius = 0.21/2;

		RobotHAL(/* args */);
		virtual ~RobotHAL();

		virtual void Tick() {};

		virtual bool IsBlueTeam() { return BlueTeam; };

		virtual bool HasConnection() { return true; };

		virtual bool IsStarted() { return true; };

		virtual void IndicateMV(bool HasMV) {};

		virtual void SendScore(int score) {};

		//Can the robot fit at this location in the terrain ?
		bool IsLocationValid(Vector2dd pos, double rot) const; 

		Vector2dd GetForwardVector();

		Vector2dd GetStoppingPosition();

		Vector2dd GetOffsetWorld(Vector2dd offset)
		{
			return position + offset.rotate(Rotation.Pos);
		}

		virtual void SetPosition(Vector2dd NewPosition, double NewRotation) {position = NewPosition; Rotation.Pos = NewRotation;};

		virtual double Rotate(double target, double &TimeBudget);

		//positive distance means forwards, negative means backwards
		virtual double LinearMove(double distance, double &TimeBudget);

		//move to position, no rotation target
		virtual double MoveTo(Vector2dd target, double &TimeBudget, ForceDirection direction = ForceDirection::None);

		bool IsOffsetInSingularity(Vector2dd target, Vector2dd offset);
		//the pair is distance, angle
		std::vector<std::pair<double, double>> GetOffsetTarget(Vector2dd target, Vector2dd offset);

		//Move the robot so that pos + rotated(offset) = target
		double MoveToOffset(Vector2dd target, Vector2dd offset, double &TimeBudget, ForceDirection direction = ForceDirection::None);

		double MovePath(Vector2dd Target, double& TimeBudget, ForceDirection direction = ForceDirection::None);

		double MovePathOffset(Vector2dd Target, Vector2dd offset, double& TimeBudget);

		//atomic claw move
		virtual double MoveClawVertical(double height, double &TimeBudget);
		virtual double MoveClawExtension(double extension, double &TimeBudget);

		virtual double MoveTray(int index, double extension, double &TimeBudget);
		
		//move the claw to height, while keeping the claw at InitialExtension during the travel
		double MoveClawTo(double height, double &TimeBudget, double InitialExtension, double FinalExtension);
	};

} // namespace overlord