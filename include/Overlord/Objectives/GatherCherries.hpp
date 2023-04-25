#pragma once

#include "Overlord/BaseObjective.hpp"


/*namespace Overlord 
{
	class Objective : public BaseObjective
	{
	public:
		//Estimate the amount of points per second that executing this objective will give
		//Return value is points gained / time estimated 
		virtual double ExecuteObjective(double &TimeBudget, RobotHAL* robot, BoardMemory* BoardState, RobotMemory* RobotState) override {return 0;};

		//Get the total number of points that this objective have given us
		virtual double GetPoints() override {return 0;};
	};
}*/

namespace Overlord 
{
	class GatherCherriesObjective : public BaseObjective
	{
	public:
		//Estimate the amount of points per second that executing this objective will give
		//Return value is points gained / time estimated 
		virtual double ExecuteObjective(double &TimeBudget, RobotHAL* robot, BoardMemory* BoardState, RobotMemory* RobotState) override;

		//Get the total number of points that this objective have given us
		virtual double GetPoints() override;
	};
}