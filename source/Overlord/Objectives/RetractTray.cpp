#include "Overlord/Objectives/RetractTray.hpp"

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

pair<double, vector<ActuatorType>> RetractTrayObjective::ExecuteObjective(double &TimeBudget, 
	RobotHAL* robot, vector<Object> &BoardState, RobotMemory* RobotState)
{
	vector<ActuatorType> act = {GetTrayType(Tray)};
	if (robot->Trays[Tray].Pos > PositionTolerance)
	{
		robot->MoveTray(Tray, 0, TimeBudget);
		return {0.001, act};
	}
	return {0, act};
}