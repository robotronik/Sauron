#pragma once

#include "Overlord/BaseObjective.hpp"


namespace Overlord 
{
	class RetractClawsObjective : public BaseObjective
	{
	public:
		//Estimate the amount of points per second that executing this objective will give
		//Return value is points gained / time estimated and the actuators used
		virtual std::pair<double, std::vector<ActuatorType>> ExecuteObjective(double &TimeBudget, 
			RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState) override;

		//Get the total number of points that this objective have given us
		virtual double GetPoints(RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState) {return 0;};

		virtual std::string GetName() {return "Retract Claws";};
	};

}