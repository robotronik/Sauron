#include "Overlord/Objectives/DepositCherries.hpp"

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

pair<double, vector<ActuatorType>> DepositCherriesObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	double BaseTime = TimeBudget;
	auto stoppos = robot->GetStoppingPosition();
	Vector2dd basketcoords = {-1.48, 0.75};
	if (robot->IsBlueTeam())
	{
		basketcoords.y = -basketcoords.y;
	}
	
	static const vector<ActuatorType> act = {ActuatorType::Wheels};
	auto parkpos = basketcoords - robot->CherryDepositPosition - Vector2dd(robot->CherryDepositPosition.x/4, 0);
	auto deltapos = parkpos-robot->position;
	if (deltapos.length()>robot->CherryDepositPosition.length()*1.5)
	{
		robot->MovePath(parkpos, TimeBudget, RobotHAL::ForceDirection::Forward);
	}
	if (TimeBudget < __DBL_EPSILON__)
	{
		return {0, act};
	}
	robot->MovePathOffset(basketcoords, robot->CherryDepositPosition, TimeBudget);
	if (TimeBudget < __DBL_EPSILON__)
	{
		return {0, act};
	}
	int nc = RobotState->Cherries.size();
	RobotState->Cherries.clear();
	double score = nc*nc/100.0;
	double timeittook = BaseTime-TimeBudget;
	return {score/timeittook, act};
}

double DepositCherriesObjective::GetPoints(RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	return 0;
}