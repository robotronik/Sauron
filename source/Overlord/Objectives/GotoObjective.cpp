#include "Overlord/Objectives/GotoObjective.hpp"

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

pair<double, vector<ActuatorType>> GotoObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	double BaseTime = TimeBudget;
	auto stoppos = robot->GetStoppingPosition();
	Vector2dd tgtcoord = {0.2, 0.0};
	
	static const vector<ActuatorType> act = {ActuatorType::Wheels};
	robot->MoveTo(tgtcoord, TimeBudget, RobotHAL::ForceDirection::Forward);
	double timeittook = BaseTime - TimeBudget;
	return {10/timeittook, act};
}

double GotoObjective::GetPoints(RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	return 0;
}