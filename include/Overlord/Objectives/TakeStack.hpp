#pragma once

#include "Overlord/BaseObjective.hpp"
#include "Overlord/BoardMemory.hpp"

#include <vector>

namespace Overlord 
{
	class TakeStackObjective : public BaseObjective
	{
	public:

		//Take or drop cakes that in the tray
		bool ClawCake(RobotHAL* robot, std::vector<Object> &BoardState, std::vector<Object>& tray, bool take);

		//move the claw to height, while keeping the claw at InitialExtension during the travel
		double MoveClawTo(RobotHAL* robot, double height, double &TimeBudget, double InitialExtension, double FinalExtension);

		double GatherIngredients(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState, ObjectType type);
		//Estimate the amount of points per second that executing this objective will give
		//Return value is points gained / time estimated 
		virtual double ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState) override;

		//Get the total number of points that this objective have given us
		virtual double GetPoints() override {return 0;};
	};
}