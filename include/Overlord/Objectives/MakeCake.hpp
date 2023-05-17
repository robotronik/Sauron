#pragma once

#include "Overlord/BaseObjective.hpp"
#include "Overlord/BoardMemory.hpp"

#include <vector>

namespace Overlord 
{
	//make single-layer cake
	class MakeCakeObjective : public BaseObjective
	{
	public:
		//Estimate the amount of points per second that executing this objective will give
		//Return value is points gained / time estimated 
		virtual std::pair<double, std::vector<ActuatorType>> ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState) override;

		//Get the total number of points that this objective have given us
		virtual double GetPoints(RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState) override {return 0;};

		virtual std::string GetName() override {return "Make cake";};
	};
}