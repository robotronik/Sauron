#include "Overlord/Objectives/MakeCake.hpp"

#include <map>
#include <vector>

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

pair<double, vector<ActuatorType>> MakeCakeObjective::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	bool InSim = TimeBudget > 50;
	double BaseTime = TimeBudget;
	int PointsMade = 0;
	vector<ActuatorType> act = {ActuatorType::Wheels, ActuatorType::Claws, ActuatorType::Tray0, ActuatorType::Tray1, ActuatorType::Tray2};
	while (TimeBudget > __DBL_EPSILON__)
	{
		//Gather Cakes
		bool HasIngredients = RobotState->GetCakeColorsStored().size() == 3;
		auto cakes = BoardState;
		FilterType(cakes, {ObjectType::CakeBrown, ObjectType::CakePink, ObjectType::CakeYellow});
		DistanceClip(cakes, robot->GetStoppingPosition() + robot->ClawPickupPosition.rotate(robot->Rotation.Pos), CakeRadius);

		int browntray = 1;
		int yellowtray = 2;
		int pinktray = 3;
		//move inside of a drop zone
		//move the claws slightly above cake height
		//drop and take
		//do it only once, don't look far

		//pink
		//yellow
		//brown

		//stage 0 : go to drop zone
		//stage 1 : take a yellow layer from tray 1
		//		-open claws if not at target
		//		-extend tray 1
		//		-move claw to tray 1 + (number of cakes-1) * cakeheight
		//		-close claw
		//		-take from tray 1
		//stage 2 : drop yellow layer on top of tray 0
		//		-closed claws if not at target
		//		-if tray 1 extended
		//			-move claw up slightly
		//			-retract tray 1
		//		-extend tray 0
		//		-move claw to tray 0 + number of cakes * cakeheight
		//		-open claw
		//		-drop cakes
		//stage 3 : take two layer from tray 0
		//		-open claw if not at target
		//		-move claw to tray 0 + (number of cakes-2) * cakeheight
		//		-close claws
		//		-take from tray 0
		//stage 4 : drop the two layers on the table
		//		-closed claws if not at target
		//		-if tray 0 extended
		//			-move claw up slightly
		//			-retract tray 0
		//		-move claw to ground
		//		-open claw
		//		-drop layers
		//stage 5 : take all pink layer from tray 2
		//		-claws open if not at height or tray 2 not extended
		//		-extend tray 2
		//		-move claws to tray 2
		//		-close claws
		//		-take from tray 2
		//stage 6 : drop the pink layer in front of the robot
		//stage 7 : pick up the remaining extra cakes and put them back in the top

		//conditions :
		//stage 1 : claws empty, no yellow cake in tray 0
		//stage 2 : single yellow cake in claws
		//stage 3 : yellow cake in the last elem of tray 0
		//stage 4 : yellow and brown in claws
		//stage 5 : two layers of yellow and brown in front of the robot, a pink in the robot
		//stage 6 : ----------------------------------------------------, pink in the bottom of the claws

		set<ObjectType> typesinfront, layersneeded;
		for (int i = 0; i < cakes.size(); i++)
		{
			typesinfront.insert(cakes[i].Type);
		}
		layersneeded = GetCakeComplement(typesinfront);
		

		bool ClawsEmpty = false;
		int stageidx = -1;
		bool firststagedone = false;
		auto& clawtray = RobotState->CakeTrays[0];
		switch (RobotState->CakeTrays[0].size())
		{
		case 0:
			ClawsEmpty = true;
			break;
		case 1:
			if (clawtray[0].Type == ObjectType::CakeYellow)
			{
				stageidx = 2;
			}
			break;
		case 2:
			if (clawtray[0].Type == ObjectType::CakeBrown && clawtray[1].Type == ObjectType::CakeYellow)
			{
				stageidx = 4;
			}
			break;
		}
		if (layersneeded.size() == 1 && layersneeded.find(ObjectType::CakePink) != layersneeded.end())
		{
			if (!ClawsEmpty && clawtray[0].Type == ObjectType::CakePink)
			{
				stageidx = 6;
			}
			else if (RobotState->CakeTrays[pinktray].size() > 0 && RobotState->CakeTrays[pinktray][0].Type == ObjectType::CakePink)
			{
				stageidx = 5;
			}
		}
		if (layersneeded.size() == 0 && cakes.size() > 3)
		{
			stageidx = 7;
		}
		if (ClawsEmpty && stageidx == -1)
		{
			auto& bottomtray = RobotState->CakeTrays[browntray];
			int bottomsize = bottomtray.size();
			auto& middletray = RobotState->CakeTrays[yellowtray];
			int middlesize = middletray.size();
			if (bottomsize >= 2 && bottomtray[bottomsize-1].Type == ObjectType::CakeYellow && bottomtray[bottomsize-2].Type == ObjectType::CakeBrown)
			{
				stageidx = 3;
			}
			else if (middlesize >= 1 && middletray[middlesize-1].Type == ObjectType::CakeYellow)
			{
				stageidx = 1;
			}
		}
		
		if (stageidx == -1)
		{
			break;
		}

		auto PickupFromTray = [&](int trayidx, int num)
		{
			int numcakes;
			if (trayidx == 0)
			{
				numcakes = cakes.size();
			}
			else
			{
				numcakes = RobotState->CakeTrays[trayidx].size();
				if (robot->Trays[trayidx-1].Pos < 1- PositionTolerance)
				{
					double parkpos = trayidx == pinktray ? robot->TrayHeights[trayidx-1] : robot->TrayHeights[trayidx] + (RobotState->CakeTrays[trayidx].size()+0.5)*CakeHeight;
					robot->MoveClawTo(parkpos, TimeBudget, 0, 0);
					robot->MoveTray(trayidx-1, 1, TimeBudget);
				}
			}
			robot->MoveClawTo(robot->TrayHeights[trayidx] + (numcakes-num)*CakeHeight + PositionTolerance, TimeBudget, 0, 1);
			if (TimeBudget < __DBL_EPSILON__)
			{
				return false;
			}
			if (trayidx == 0)
			{
				RobotState->ClawCake(robot, BoardState, true);
			}
			else
			{
				RobotState->TransferCake(robot, trayidx, false);
			}
			return true;
		};

		auto DepositToTray = [&](int prevtray, int trayidx)
		{
			if (prevtray > 0 && robot->Trays[prevtray-1].Pos > PositionTolerance)
			{
				robot->MoveClawTo(robot->TrayHeights[prevtray] + (RobotState->CakeTrays[prevtray].size()+0.5)*CakeHeight, TimeBudget, 1, 1);
				robot->MoveTray(prevtray-1, 0, TimeBudget);
			}
			int numcakes;
			if (trayidx > 0)
			{
				robot->MoveTray(trayidx-1, 1, TimeBudget);
				numcakes = RobotState->CakeTrays[trayidx].size();
			}
			else
			{
				numcakes = cakes.size();
			}
			robot->MoveClawTo(robot->TrayHeights[trayidx] + numcakes*CakeHeight + PositionTolerance, TimeBudget, 1, 0);
			if (TimeBudget < __DBL_EPSILON__)
			{
				return false;
			}
			if (trayidx>0)
			{
				RobotState->TransferCake(robot, trayidx, true);
			}
			else
			{
				RobotState->ClawCake(robot, BoardState, false);
			}
			return true;
		};
		
		bool donenothing = false;
		switch (stageidx)
		{
		case 1:
			{
				PickupFromTray(yellowtray, 1);
			}
			break;
		case 2:
			{
				DepositToTray(yellowtray, browntray);
			}
			break;
		case 3:
			{
				PickupFromTray(browntray, 2);
			}
			break;
		case 4:
			{
				if (DepositToTray(browntray, 0))
				{
					PointsMade+=2;
				}
			}
			break;
		case 5:
			{
				PickupFromTray(pinktray, RobotState->CakeTrays[pinktray].size());
			}
			break;
		case 6:
			{
				if(DepositToTray(pinktray, 0))
				{
					PointsMade+=2;
				}
			}
			break;
		case 7:
			{
				int layersleft = cakes.size()-3;
				if(PickupFromTray(0, cakes.size()-3))
				{
					PointsMade+=layersleft;
				}
			}
			break;
		default:
			donenothing = true;
			break;
		}
		if (donenothing)
		{
			break;
		}
		
	}
	
	if (PointsMade == 0)
	{
		return {0, act};
	}
	

	return {PointsMade/(BaseTime-TimeBudget), act};
}