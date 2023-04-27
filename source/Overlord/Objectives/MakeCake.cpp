#include "Overlord/Objectives/MakeCake.hpp"

#include <map>
#include <vector>

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

bool MakeCakeObjective::ClawCake(RobotHAL* robot, BoardMemory* BoardState, vector<Object>& tray, bool take)
{
	Vector2d<double> clawposloc{0.1,0};
	Vector2d<double> clawpos = robot->position+Vector2d<double>(robot->Rotation.Pos)*clawposloc.x + Vector2d<double>(robot->Rotation.Pos + M_PI_2)*clawposloc.y;
	if (take)
	{

		for (int i = BoardState->ObjectsOnBoard.size()-1; i >=0 ; i--)
		{
			Object &object = BoardState->ObjectsOnBoard[i];
			if (object.Type != ObjectType::CakeBrown && object.Type != ObjectType::CakePink && object.Type != ObjectType::CakeYellow)
			{
				continue;
			}
			
			if ((clawpos-object.position).length() < 0.01) //tolerance here
			{
				tray.push_back(object);
				BoardState->ObjectsOnBoard.erase(BoardState->ObjectsOnBoard.begin() + i);
			}
		}
	}
	else
	{
		for (int i = 0; i < tray.size(); i++)
		{
			Object &object = tray[i];
			object.position = clawpos;
			BoardState->ObjectsOnBoard.push_back(object);
		}
		tray.clear();
	}
	return true;
}

double MakeCakeObjective::MoveClawTo(RobotHAL* robot, double height, double &TimeBudget, double InitialExtension, double FinalExtension)
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

double MakeCakeObjective::GatherIngredients(double &TimeBudget, RobotHAL* robot, BoardMemory* BoardState, RobotMemory* RobotState, ObjectType type)
{
	return 0;
}

double MakeCakeObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, BoardMemory* BoardState, RobotMemory* RobotState)
{
	const vector<double> trayHeights = {0, 0.02, 0.1, 0.18};
	while (TimeBudget > __DBL_EPSILON__)
	{
		//Gather Cakes
		map<ObjectType, int> NeededCakes = {{ObjectType::CakeBrown, 3}, {ObjectType::CakeYellow, 3}, {ObjectType::CakePink, 3}};
		vector<bool> DoneCakes(4, false);
		bool AnyDone = false;
		for (int i = 0; i < 4; i++)
		{
			auto& tray = RobotState->CakeTrays[i];
			bool hascorrectcake = true;
			static const vector<ObjectType> correctOrder = {ObjectType::CakeBrown, ObjectType::CakeYellow, ObjectType::CakePink};
			for (int j = 0; j < tray.size(); j++)
			{
				NeededCakes[tray[j].Type] -= 1;
				if (j>=correctOrder.size())
				{
					continue;
				}
				if (tray[j].Type != correctOrder[j])
				{
					hascorrectcake = false;
				}
				
			}
			if (tray.size() == 3)
			{
				//full : need to drop
				DoneCakes[i] = hascorrectcake;
				AnyDone = true;
			}
		}
		bool CakeInClawsDone = DoneCakes[0];
		bool ClawsEmpty = RobotState->CakeTrays[0].size() == 0;

		if (!ClawsEmpty)
		{
			if (CakeInClawsDone)
			{
				double clawBudget = TimeBudget;
				double movebudget = TimeBudget;
				if (abs(robot->ClawHeight.Pos-trayHeights[0]) > 1e-6)
				{
					robot->MoveClaw(robot->ClawHeight.Pos, 1, clawBudget);
				}
				double minbudget = clawBudget;
				for (int i = 0; i < 3; i++) //retract all trays at once
				{
					double localbudget = clawBudget;
					robot->MoveTray(i, 0, localbudget);
					if (localbudget < minbudget)
					{
						minbudget = localbudget;
					}
				}
				clawBudget = minbudget;
				
				robot->ClawHeight.SetTarget(trayHeights[0]); // lower the claws
				robot->ClawHeight.Tick(clawBudget);
				ObjectType dropzonetargettype = robot->IsBlueTeam() ? ObjectType::BlueDropZone : ObjectType::GreenDropZone;
				auto zones = BoardState->FindObjectsSorted((uint32_t)dropzonetargettype, robot->GetStoppingPosition());
				robot->MoveTo(zones[0].position, movebudget);
				double budgetleft = min(clawBudget, movebudget);
				TimeBudget = budgetleft;
				if (TimeBudget < __DBL_EPSILON__)
				{
					break;
				}
				robot->ClawExtension.SetTarget(0); //open the claws
				robot->ClawExtension.Tick(TimeBudget);
				//drop the cake
				if (TimeBudget < __DBL_EPSILON__)
				{
					break;
				}
				ClawCake(robot, BoardState, RobotState->CakeTrays[0], false);
			}
			else
			{
				int traythatneedsthecake = 0;
				ObjectType caketype = RobotState->CakeTrays[0][0].Type;
				for (int i = 1; i < 4; i++)
				{
					bool needsit = true;
					for (int j = 0; j < RobotState->CakeTrays[i].size(); j++)
					{
						if (RobotState->CakeTrays[i][j].Type == caketype)
						{
							needsit = false;
							break;
						}
					}
					if (needsit)
					{
						traythatneedsthecake = i;
						break;
					}
				}
				if (abs(robot->ClawHeight.Pos-trayHeights[traythatneedsthecake]) > 1e-6)
				{
					robot->MoveClaw(robot->ClawHeight.Pos, 1, TimeBudget);
				}
				
			}
		}
	}
	
	return 0;
}