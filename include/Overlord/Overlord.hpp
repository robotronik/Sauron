#pragma once

#include <vector>
#include <memory>
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

		double TimeLeft;

		#ifdef WITH_SAURON
		BoardGL visualiser;
		#endif

		void Init();

		void GatherData(); //Update memories

		void Run();

		bool Display();

		void Thread();
	};
}