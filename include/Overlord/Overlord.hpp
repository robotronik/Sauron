#pragma once

#include <vector>
#include <memory>
#include <chrono>
#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
#include "Overlord/BaseObjective.hpp"

#ifdef WITH_SAURON
#include "visualisation/BoardGL.hpp"
#endif
namespace Overlord
{
	class Manager
	{
	public:
		std::vector<Object> PhysicalBoardState;
		std::vector<RobotMemory> PhysicalRobotStates;
		std::vector<RobotHAL*> RobotControllers; 
		std::vector<std::unique_ptr<BaseObjective>> Objectives;

		std::chrono::time_point<std::chrono::steady_clock> lastTick;
		double TimeLeft;

		#ifdef WITH_SAURON
		BoardGL visualiser;
		#endif

		void Init();

		void GatherData(); //Update memories

		void Run(double delta);

		bool Display();

		void Thread();
	};
}