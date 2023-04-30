#include "Overlord/BoardMemory.hpp"
#include <algorithm>
#include <iostream>
#include "Overlord/RobotHAL.hpp"

using namespace std;
using namespace Overlord;



int RobotMemory::ClawCake(RobotHAL* robot, std::vector<Object> &BoardState, bool take)
{
	Vector2dd clawpos = robot->GetOffsetWorld(robot->ClawPickupPosition);
	int numtransfered = 0;
	if (take)
	{
		int couldhavetaken = 0;
		int numtoskip = robot->ClawHeight.Pos/CakeHeight - __DBL_EPSILON__;
		for (int i = BoardState.size()-1; i >=0 ; i--)
		{
			Object &object = BoardState[i];
			if (!object.IsCake())
			{
				continue;
			}
			if ((clawpos-object.position).length() > CakeTolerance)
			{
				continue;
			}
			if (couldhavetaken < numtoskip)
			{
				couldhavetaken++;
				cout << "Skipping taking a cake from the board because the claw was too high" << endl;
				continue;
			}
			object.position = {0.01, 0}; //not correctly inserted yet
			CakeTrays[0].push_back(object);
			BoardState.erase(BoardState.begin() + i);
			numtransfered++;
		}
	}
	else
	{
		for (int i = 0; i < CakeTrays[0].size(); i++)
		{
			Object &object = CakeTrays[0][i];
			object.position = clawpos;
			BoardState.push_back(object);
			numtransfered++;
		}
		CakeTrays[0].clear();
	}
	if (numtransfered == 0)
	{
		cerr << "WARNING : Tried to take cakes, but none were present" <<endl;
	}
	
	return numtransfered;
}

bool RobotMemory::TransferCake(RobotHAL* robot, int trayidx, bool ToTray)
{
	assert(trayidx != 0);
	assert(trayidx < sizeof(CakeTrays)/sizeof(CakeTrays[0]));
	if (robot->Trays[trayidx-1].Pos < 1-PositionTolerance)
	{
		cerr << "WARNING : Transferring to tray but tray is not deployed !" <<endl;
	}
	if (robot->ClawHeight.Pos < robot->TrayHeights[trayidx])
	{
		cerr << "WARNING : Claws are too low to be able to transfer !" <<endl;
	}
	double CakeFillHeight = CakeTrays[trayidx].size() * CakeHeight;
	double deltaHeight = robot->ClawHeight.Pos - robot->TrayHeights[trayidx];
	if (deltaHeight < CakeFillHeight && ToTray)
	{
		cerr << "WARNING : Deposited cakes will collide with cakes already present" << endl;
	}
	if (deltaHeight < 0)
	{
		cerr << "WARNING : Claws are below the tray" <<endl;
	}
	if (deltaHeight > CakeFillHeight && !ToTray)
	{
		cerr << "ERROR : Claws too high to take from tray by " << deltaHeight-CakeFillHeight << "m" <<endl;
		return false;
	}
	if (ToTray)
	{
		for (int i = 0; i < CakeTrays[0].size(); i++)
		{
			CakeTrays[trayidx].push_back(CakeTrays[0][i]);
		}
		CakeTrays[0].clear();
	}
	else
	{
		int NumToSkip = max(deltaHeight/CakeHeight, 0.0);
		for (int i = NumToSkip; i < CakeTrays[trayidx].size(); i++)
		{
			CakeTrays[0].push_back(CakeTrays[trayidx][i]);
		}
		CakeTrays[trayidx].resize(NumToSkip);
	}
	return true;
	
}

vector<Object> Overlord::FindObjects(const std::vector<Object> &in, uint TypeFilter)
{
	vector<Object> outvec;
	outvec.reserve(in.size());
	for (const auto &obj : in)
	{
		if ((uint)obj.Type & TypeFilter)
		{
			outvec.push_back(obj);
		}
	}
	return outvec;
}

vector<Object> Overlord::FindObjectsSorted(const std::vector<Object> &in, uint TypeFilter, Vector2dd SearchPos)
{
	vector<Object> outvec = FindObjects(in, TypeFilter);
	sort(outvec.begin(), outvec.end(), [SearchPos](Object a, Object b)
	{
		return (SearchPos-a.position).lengthsquared()<(SearchPos-b.position).lengthsquared();
	});
	return outvec;
}

std::optional<Vector2dd> Overlord::FindNearestCakeStack(const std::vector<Object> &in, Vector2dd SearchPos)
{
	std::optional<Vector2dd> closest = nullopt;
	for (int i = 0; i < in.size(); i++)
	{
		auto& cake0 = in[i];
		Vector2dd mean = cake0.position;
		int numcakes = 1;
		if (!cake0.IsCake())
		{
			//not cake
			continue;
		}
		for (int j = i+1; j < in.size(); j++)
		{
			auto& caken = in[j];
			if (!caken.IsCake())
			{
				//not cake
				continue;
			}
			if ((caken.position-cake0.position).length() < CakeTolerance) //add tolerance here
			{
				numcakes++;
				mean += caken.position;
			}
		}
		mean/=numcakes;
		if (numcakes < 3)
		{
			continue;
		}
		if (closest.has_value())
		{
			if ((SearchPos-closest.value()).length() < (SearchPos-mean).length()) //farther than the best
			{
				continue;
			}
		}
		closest = mean;
	}
	return closest;
}