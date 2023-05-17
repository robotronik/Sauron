#pragma once

#include <vector>
#include <string>
#include <utility>

#include "Overlord/Objectives/Actuators.hpp"

namespace Overlord 
{
	class RobotHAL;
	struct Object;
	class RobotMemory;
	class BaseObjective
	{
	public:

		//Estimate the amount of points per second that executing this objective will give
		//Return value is points gained / time estimated and the actuators used
		virtual std::pair<double, std::vector<ActuatorType>> ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState) 
		{return std::make_pair(-1.0, std::vector<ActuatorType>());};

		//Get the total number of points that this objective have given us
		virtual double GetPoints(RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState) {return 0;};

		virtual std::string GetName() {return "Base Objective";};
	};

}
