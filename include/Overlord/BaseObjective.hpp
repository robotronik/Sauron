#pragma once


namespace Overlord 
{
	class RobotHAL;
	class BoardMemory;
	class RobotMemory;
	class BaseObjective
	{
	public:

		//Estimate the amount of points per second that executing this objective will give
		//Return value is points gained / time estimated 
		virtual double ExecuteObjective(double &TimeBudget, RobotHAL* robot, BoardMemory* BoardState, RobotMemory* RobotState) {return -1;};

		//Get the total number of points that this objective have given us
		virtual double GetPoints() {return 0;};
	};

}
