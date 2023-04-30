#include "Overlord/Objectives/TakeStack.hpp"

#include <map>
#include <vector>

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

pair<double, vector<ActuatorType>> TakeStackObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	bool InSim = TimeBudget > 50;
	double BaseTime = TimeBudget;
	int NumTaken =0;
	static const vector<ActuatorType> act = {ActuatorType::Wheels, ActuatorType::Claws};
	while (TimeBudget > __DBL_EPSILON__)
	{
		//Gather Cakes
		vector<bool> DoneCakes(4, false);
		bool AnyDone = false;
		int NextEmpty = -1;
		for (int i = 1; i < 4; i++)
		{
			if (RobotState->CakeTrays[i].size() == 0)
			{
				NextEmpty = i;
				break;
			}
			
		}

		if (NextEmpty == -1)
		{
			break;
		}
		
		bool ClawsEmpty = RobotState->CakeTrays[0].size() == 0;
		auto nearestStack = FindNearestCakeStack(BoardState, robot->GetStoppingPosition() + robot->ClawPickupPosition.rotate(robot->Rotation.Pos));
		if (ClawsEmpty && nearestStack.has_value())
		{
			
			double clawbudget = TimeBudget;
			double movebudget = TimeBudget;
			if ((nearestStack.value()-robot->GetOffsetWorld(robot->ClawPickupPosition)).length() > PositionTolerance)
			{
				robot->MoveToOffset(nearestStack.value(), robot->ClawPickupPosition, movebudget);
				auto clawpos = robot->GetOffsetWorld(robot->ClawPickupPosition);
				double distancetotarget = (clawpos-nearestStack.value()).length();
				assert(distancetotarget < PositionTolerance || movebudget < __DBL_EPSILON__);
				robot->MoveClawExtension(0, clawbudget);
				robot->MoveClawVertical(robot->TrayHeights[0], clawbudget);
			}
			TimeBudget = min(clawbudget, movebudget);
			if (TimeBudget < __DBL_EPSILON__)
			{
				return {0, act};
			}

			robot->MoveClawTo(robot->TrayHeights[0], TimeBudget, 0, 1);
			if (TimeBudget < __DBL_EPSILON__)
			{
				return {0, act};
			}
			NumTaken += RobotState->ClawCake(robot, BoardState, true);
			break; //do it only once, don't look far
		}
		else
		{
			break;
		}
		
		
	}
	
	return {NumTaken/(BaseTime-TimeBudget), act};
}