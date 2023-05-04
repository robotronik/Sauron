
#pragma once

#include "Overlord/RobotHAL.hpp"

#include <string>
#include <chrono>

class serialib;

namespace Overlord
{
	class RobotHandle : public RobotHAL
	{
		char receivebuffer[1024] = {0};
		int numreceived = 0;
		bool RipCordStatus = true;
		bool keepalivestatus = false;
		std::chrono::time_point<std::chrono::steady_clock> lastTick;
		Vector2dd RobotReportedPosition = {0,0};
		double RobotReportedRotation = 0;
		Vector2dd RobotOrigin = {0,0};
		double RobotRotationOffset = 0;
		int LastRotate = 0;
		Vector2d<int> LastPos = {0,0};
	public:
		serialib* bridgehandle;
		RobotHandle(serialib* InBridge = nullptr);
		virtual ~RobotHandle();

		virtual void Tick() override;

		virtual bool HasConnection() override;

		virtual bool IsStarted() override;

		void ToRobotPosition(Vector2dd& pos, double& rot);

		void ToWorldPosition(Vector2dd& pos, double& rot);

		virtual void SetPosition(Vector2dd NewPosition, double NewRotation) override;

		virtual double Rotate(double target, double &TimeBudget) override;

		//positive distance means forwards, negative means backwards
		virtual double LinearMove(double distance, double &TimeBudget) override;

		virtual double MoveTo(Vector2dd target, double &TimeBudget, ForceDirection direction) override;

		virtual double MoveClawVertical(double height, double &TimeBudget) override;
		virtual double MoveClawExtension(double extension, double &TimeBudget) override;

		virtual double MoveTray(int index, double extension, double &TimeBudget) override;
	};
	
	
	
} // namespace Overlord
