#include "Overlord/Objectives/TakeStack.hpp"

#include <map>
#include <vector>

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

bool TakeStackObjective::ClawCake(RobotHAL* robot, std::vector<Object> &BoardState, vector<Object>& tray, bool take)
{
	Vector2d<double> clawpos = robot->position+robot->ClawPickupPosition.rotate(robot->Rotation.Pos);
	if (take)
	{
		for (int i = BoardState.size()-1; i >=0 ; i--)
		{
			Object &object = BoardState[i];
			if (object.Type != ObjectType::CakeBrown && object.Type != ObjectType::CakePink && object.Type != ObjectType::CakeYellow)
			{
				continue;
			}
			
			if ((clawpos-object.position).length() < 0.01) //tolerance here
			{
				object.position = {0.1, 0};
				tray.push_back(object);
				BoardState.erase(BoardState.begin() + i);
			}
		}
	}
	else
	{
		for (int i = 0; i < tray.size(); i++)
		{
			Object &object = tray[i];
			object.position = clawpos;
			BoardState.push_back(object);
		}
		tray.clear();
	}
	return true;
}

double TakeStackObjective::MoveClawTo(RobotHAL* robot, double height, double &TimeBudget, double InitialExtension, double FinalExtension)
{
	//if not at the target, close the claws
	if (abs(robot->ClawHeight.Pos-height) > 1e-6)
	{
		robot->MoveClaw(robot->ClawHeight.Pos, InitialExtension, TimeBudget);
	}
	double minbudget = TimeBudget;
	for (int i = 0; i < 3; i++) //retract all trays at once
	{
		double localbudget = TimeBudget;
		robot->MoveTray(i, 0, localbudget);
		if (localbudget < minbudget)
		{
			minbudget = localbudget;
		}
	}
	TimeBudget = minbudget;
	robot->MoveClaw(height, InitialExtension, TimeBudget);
	robot->MoveClaw(height, FinalExtension, TimeBudget);
	return TimeBudget;
}

double TakeStackObjective::GatherIngredients(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState, ObjectType type)
{
	return 0;
}

double TakeStackObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	double BaseTime = TimeBudget;
	int NumTaken =0;
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

		if (ClawsEmpty)
		{
			double clawbudget = TimeBudget;
			double movebudget = TimeBudget;
			MoveClawTo(robot, robot->TrayHeights[0], clawbudget, 0, 0); //move claw open to bottom
		}
	}
	
	return NumTaken/(BaseTime-TimeBudget);
}