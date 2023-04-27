
#pragma once

#include "Overlord/RobotHAL.hpp"

#include <string>

class serialib;

namespace Overlord
{
	class RobotHandle : public RobotHAL
	{
		char receivebuffer[1024] = {0};
		int numreceived = 0;
	public:
		serialib* bridgehandle;
		RobotHandle(serialib* InBridge = nullptr);
		virtual ~RobotHandle();

		virtual void Tick() override;

		virtual double Rotate(double target, double &TimeBudget) override;

		//positive distance means forwards, negative means backwards
		virtual double LinearMove(double distance, double &TimeBudget) override;
	};
	
	
	
} // namespace Overlord
