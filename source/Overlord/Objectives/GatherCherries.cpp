#include "Overlord/Objectives/GatherCherries.hpp"

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"

double Overlord::GatherCherriesObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, BoardMemory* BoardState, RobotMemory* RobotState)
{
	double spx, spy;
	double BaseTime = TimeBudget;
	robot->GetStoppingPosition(spx, spy);
	const auto cherries = BoardState->FindObjectsSorted((uint8_t)ObjectType::Cherry, spx, spy);
	if (cherries.size() == 0)
	{
		return 0;
	}
	//vacuum to the right of the robot
	//robot width is 310mm
	
	robot->MoveTo(cherries[0].PosX, cherries[0].PosY, TimeBudget); //TODO: move so that the intake is that the cherry position, not the robot
	if (TimeBudget < __DBL_EPSILON__)
	{
		return 0; //not able to get to the cherry in time, 0 points
	}
	//TODO: take the cherry
	for (int i = 0; i < BoardState->ObjectsOnBoard.size(); i++)
	{
		if (BoardState->ObjectsOnBoard[i] == cherries[0])
		{
			BoardState->ObjectsOnBoard.erase(BoardState->ObjectsOnBoard.begin()+i);
			break;
		}
	}
	RobotState->Cherries.push_back(cherries[0]);

	double timeittook = BaseTime-TimeBudget;
	return 1/timeittook; //we'll say it gives one point, even though it doesn't
}

double Overlord::GatherCherriesObjective::GetPoints()
{
	return 0;
}