#include "Overlord/Objectives/GoHomeObjective.hpp"

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

Vector2dd GoHomeObjective::GetNearestZone(RobotHAL* robot)
{
	auto targets = GetDropZones(robot->IsBlueTeam());
	double bestdist = INFINITY;
	int bestidx = 0;
	Vector2dd reference = robot->GetStoppingPosition();
	for (int i = 0; i < targets.size(); i++)
	{
		double dist = (reference-targets[i]).lengthsquared();
		if (dist < bestdist)
		{
			bestdist = dist;
			bestidx = i;
		}
	}
	return targets[bestidx];
}

pair<double, vector<ActuatorType>> GoHomeObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	double BaseTime = TimeBudget;
	auto target = GetNearestZone(robot);
	
	static const vector<ActuatorType> act = {ActuatorType::Wheels};
	robot->MovePath(target, TimeBudget);
	double timeleft = max(TimeBudget, 0.1);
	
	return {15/timeleft/timeleft, act};
}

double GoHomeObjective::GetPoints(RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	auto target = GetNearestZone(robot);
	if ((robot->position-target).abs().max() < ZoneWidth/2)
	{
		return 15;
	}
	return 0;
}