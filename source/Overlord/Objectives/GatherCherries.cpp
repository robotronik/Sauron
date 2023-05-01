#include "Overlord/Objectives/GatherCherries.hpp"

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

pair<double, vector<ActuatorType>> GatherCherriesObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	double BaseTime = TimeBudget;
	auto stoppos = robot->GetStoppingPosition() + robot->CherryPickupPosition.rotate(robot->Rotation.Pos);
	auto cherries = BoardState;
	FilterType(cherries, {ObjectType::Cherry});
	DistanceSort(cherries, stoppos);
	static const vector<ActuatorType> act = {ActuatorType::Wheels};
	if (cherries.size() == 0 || RobotState->Cherries.size() >= 10)
	{
		return {0, act};
	}
	
	robot->MoveToOffset(cherries[0].position, robot->CherryPickupPosition, TimeBudget); //TODO: move so that the intake is that the cherry position, not the robot
	if (TimeBudget < __DBL_EPSILON__)
	{
		return {0, act}; //not able to get to the cherry in time, 0 points
	}
	for (int i = 0; i < BoardState.size(); i++)
	{
		if (BoardState[i] == cherries[0])
		{
			BoardState.erase(BoardState.begin()+i);
			break;
		}
	}
	RobotState->Cherries.push_back(cherries[0]);

	double timeittook = BaseTime-TimeBudget;
	return {1/timeittook, act}; //we'll say it gives one point, even though it doesn't
}

double GatherCherriesObjective::GetPoints()
{
	return 0;
}