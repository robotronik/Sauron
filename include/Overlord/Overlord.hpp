#pragma once

#include <vector>
#include <set>
#include <memory>
#include <chrono>
#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
#include "Overlord/BaseObjective.hpp"

#include "data/metadata.hpp"
#include "TrackedObjects/ObjectIdentity.hpp"
#include "data/senders/Transport/TCPTransport.hpp"

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
		std::set<BaseObjective*> LastTickObjectives;
		CDFRTeam Team;

		std::chrono::time_point<std::chrono::steady_clock> lastTick;
		std::optional<std::chrono::time_point<std::chrono::steady_clock>> StartTime;

		std::unique_ptr<TCPTransport> receiver;
		char receivebuffer[1<<16];
		char* receiveptr = nullptr;

		#ifdef WITH_SAURON
		BoardGL visualiser;
		#endif

		void Init(bool simulate = false);

		void UpdateCollision();

		void SimulateEnnemy();

		void GatherData(); //Update memories

		void Run(double delta);

		bool Display(bool v3d = true);

		void Thread(bool v3d = true, bool simulate = false);
	};
}