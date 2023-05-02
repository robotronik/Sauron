#include "Overlord/Objectives/TrayRoutine.hpp"

#include <map>
#include <vector>

#include "Overlord/BoardMemory.hpp"
#include "Overlord/RobotHAL.hpp"
using namespace Overlord;
using namespace std;

#define CheckTimeBudget(points, actuators) if(TimeBudget < __DBL_EPSILON__) {return make_pair(points/(BaseTime-TimeBudget), actuators);}

pair<double, vector<ActuatorType>> TrayRoutine::ExecuteObjective(double &TimeBudget, RobotHAL* robot, std::vector<Object> &BoardState, RobotMemory* RobotState)
{
	bool InSim = TimeBudget > 50;
	double BaseTime = TimeBudget;
	while (TimeBudget > __DBL_EPSILON__)
	{
		//Gather Cakes
		vector<bool> NeedsPush(4, false);
		bool AnyDone = false;
		int NextPush = -1; //1 to 4
		for (int i = 1; i < 4; i++)
		{
			auto & tray = RobotState->CakeTrays[i];
			for (int j = 0; j < tray.size(); j++)
			{
				if (tray[j].position.length() > PositionTolerance)
				{
					NeedsPush[i] = true;
					break;
				}
			}
			if (NextPush == -1 && NeedsPush[i] && i >0)
			{
				NextPush = i;
			}
		}

		static const vector<ObjectType> TrayColors = {ObjectType::CakeBrown, ObjectType::CakeYellow, ObjectType::CakePink};
		
		bool ClawsEmpty = RobotState->CakeTrays[0].size() == 0;
		
		if (!ClawsEmpty)
		{
			ObjectType ClawColor = IsSingleColorStack(RobotState->CakeTrays[0]);
			if (ClawColor == ObjectType::Unknown)
			{
				return {0, {}}; //not single color
			}
			
			auto StoreTray = std::find(TrayColors.begin(), TrayColors.end(), ClawColor);
			int StoreIdx = StoreTray - TrayColors.begin() +1;
			if (RobotState->CakeTrays[0].size() + RobotState->CakeTrays[StoreIdx].size() > 3) //can't store : already full
			{
				return {0, {}};
			}
			bool deposit = true;
			for (auto &o : RobotState->CakeTrays[0])
			{
				if (o.position.length() < PositionTolerance)
				{
					deposit = false;
				}
			}
			if (!deposit)
			{
				return {0, {}};
			}
			
			
			
			double targetheight = robot->TrayHeights[StoreIdx] + PositionTolerance;
			vector<ActuatorType> act = {ActuatorType::Claws, GetTrayType(StoreIdx-1)};
			if (abs(robot->ClawHeight.Pos-targetheight) > PositionTolerance)
			{
				robot->MoveClawTo(targetheight, TimeBudget, 1, 1);
				CheckTimeBudget(1, act);
			}
			
			robot->MoveTray(StoreIdx-1, 1, TimeBudget); //extend tray
			CheckTimeBudget(1, act)
			robot->MoveClawTo(robot->ClawHeight.Pos, TimeBudget, 1, 0); //retract claw
			CheckTimeBudget(1, act)
			RobotState->TransferCake(robot, StoreIdx, true);
			return make_pair(1,act);
		}
		else if (ClawsEmpty && NextPush != -1)
		{
			//open claws, change height to clear tray, go back in front of tray, then move the tray forward a bit
			vector<ActuatorType> act = {ActuatorType::Claws, GetTrayType(NextPush-1)};
			auto& traypos = robot->Trays[NextPush-1];
			auto& clawheight = robot->ClawHeight;
			auto& clawext = robot->ClawExtension;
			double parkposition = NextPush == robot->TrayHeights.size()-1 ? robot->TrayHeights[NextPush-1] : robot->TrayHeights[NextPush+1];
			double deltapark = abs(clawheight.Pos-parkposition);

			//if open and claw at the top or open and claw anywhere
			if ((traypos.Pos > PositionTolerance && deltapark < PositionTolerance) || traypos.Pos > 0.5) //extended, move claw out of the way then retract the tray
			{
				if (deltapark > PositionTolerance)
				{
					robot->MoveClawTo(parkposition, TimeBudget, 0, 0);
					CheckTimeBudget(1, act)
					
				}
				robot->MoveTray(NextPush-1, 0, TimeBudget);
				CheckTimeBudget(1, act)
			}
			robot->MoveClawTo(robot->TrayHeights[NextPush], TimeBudget, 0, 1);
			CheckTimeBudget(1, act)
			robot->MoveTray(NextPush-1, 0.2, TimeBudget); // todo: set push amount here
			CheckTimeBudget(1, act)
			for (int i = 0; i < RobotState->CakeTrays[NextPush].size(); i++)
			{
				RobotState->CakeTrays[NextPush][i].position = {0.0,0.0};
			}
			return make_pair(1.0, act);
		}
		else
		{
			return {0, {}};
		}
	}
	
	return {0, {}};
}