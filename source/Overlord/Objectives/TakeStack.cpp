#include "Overlord/Objectives/TakeStack.hpp"

#include <map>
#include <vector>
#include <set>

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
		set<ObjectType> ColorsNeeded = RobotState->GetCakeColorsNeeded();
		
		bool ClawsEmpty = RobotState->CakeTrays[0].size() == 0;
		auto cakes = BoardState;
		FilterType(cakes, GetCakeComplement({}));
		DistanceSort(cakes, robot->GetStoppingPosition() + robot->ClawPickupPosition.rotate(robot->Rotation.Pos));
		auto Stacks = FindCakeStacks(cakes);
		if (ClawsEmpty && Stacks.size() > 0)
		{
			vector<Object> neareststack;
			for (auto &stack : Stacks)
			{
				ObjectType stackColor = IsSingleColorStack(stack);
				if (ColorsNeeded.find(stackColor) != ColorsNeeded.end())
				{
					neareststack = stack;
					break;
				}
			}
			if (neareststack.size() != 3)
			{
				break;
			}
			
			auto nspos = neareststack[0].position;
			double clawbudget = TimeBudget;
			double movebudget = TimeBudget;
			if ((nspos-robot->GetOffsetWorld(robot->ClawPickupPosition)).length() > PositionTolerance)
			{
				robot->MoveToOffset(nspos, robot->ClawPickupPosition, movebudget);
				auto clawpos = robot->GetOffsetWorld(robot->ClawPickupPosition);
				double distancetotarget = (clawpos-nspos).length();
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