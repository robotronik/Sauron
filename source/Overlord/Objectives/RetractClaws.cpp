#include "Overlord/Objectives/RetractClaws.hpp"

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

pair<double, vector<ActuatorType>> RetractClawsObjective::ExecuteObjective(double &TimeBudget, 
	RobotHAL* robot, vector<Object> &BoardState, RobotMemory* RobotState)
{
	static const vector<ActuatorType> act = {ActuatorType::Claws};
	if (robot->ClawExtension.Pos > PositionTolerance)
	{
		robot->MoveClawExtension(0, TimeBudget);
		return {0.001, act};
	}
	return {0, act};
}