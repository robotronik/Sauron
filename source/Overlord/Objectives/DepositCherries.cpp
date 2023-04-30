#include "Overlord/Objectives/DepositCherries.hpp"

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

pair<double, vector<ActuatorType>> DepositCherriesObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	double BaseTime = TimeBudget;
	auto stoppos = robot->GetStoppingPosition();
	Vector2dd basketcoords = {-1.5, 0.7};
	if (robot->IsBlueTeam())
	{
		basketcoords.y = -basketcoords.y;
	}
	
	static const vector<ActuatorType> act = {ActuatorType::Wheels};
	auto parkpos = basketcoords - robot->CherryDepositPosition;
	auto deltapos = parkpos-robot->position;
	if (deltapos.length()>robot->CherryDepositPosition.length()*1.5)
	{
		robot->MoveTo(basketcoords - robot->CherryDepositPosition, TimeBudget, RobotHAL::ForceDirection::Forward);
	}
	if (TimeBudget < __DBL_EPSILON__)
	{
		return {0, act};
	}
	robot->MoveToOffset(basketcoords, robot->CherryDepositPosition, TimeBudget, RobotHAL::ForceDirection::Backwards);
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

double DepositCherriesObjective::GetPoints()
{
	return 0;
}