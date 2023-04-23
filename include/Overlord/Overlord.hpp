#pragma once

#include <vector>
#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
#include "Overlord/BaseObjective.hpp"


namespace Overlord
{
	class Manager
	{
	public:
		BoardMemory PhysicalBoardState;
		std::vector<RobotMemory> PhysicalRobotStates;
		std::vector<RobotHAL*> RobotControllers; 
		std::vector<BaseObjective*> Objectives;

		void Init();

		void GatherData(); //Update memories

		void Run();

		void Display();

		void Thread();
	};
}