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
		auto couldtake = [&](int index)
		{
			const Object &object = BoardState[index];
			if (!object.IsCake())
			{
				return false;
			}
			if ((clawpos-object.position).length() > CakeTolerance)
			{
				return false;
			}
			return true;
		};
		int startidx;
		for (startidx = 0; startidx < BoardState.size() && couldhavetaken < numtoskip; startidx++)
		{
			if(couldtake(startidx))
			{
				couldhavetaken++;
				cout << "Skipping taking a cake from the board because the claw was too high" << endl;
				continue;
			}
			
		}
		for (int i = BoardState.size() - 1; i >= startidx; i--)
		{
			if(couldtake(i))
			{
				Object &object = BoardState[i];
				object.position = {0.01, 0}; //not correctly inserted yet
				object.Rot -= robot->Rotation.Pos;
				CakeTrays[0].push_back(object);
				BoardState.erase(BoardState.begin() + i);
				numtransfered++;
			}
		}
		
	}
	else
	{
		for (int i = 0; i < CakeTrays[0].size(); i++)
		{
			Object &object = CakeTrays[0][i];
			object.position = clawpos;
			object.Rot = robot->Rotation.Pos;
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

set<ObjectType> RobotMemory::GetCakeColorsStored()
{
	set<ObjectType> ColorsGot;
	for (int i = 0; i < 4; i++)
	{
		auto& tray = CakeTrays[i];
		for (int j = 0; j < tray.size(); j++)
		{
			ColorsGot.insert(tray[j].Type);
		}
	}
	return ColorsGot;
}

set<ObjectType> RobotMemory::GetCakeColorsNeeded()
{
	return GetCakeComplement(GetCakeColorsStored());
}

ObjectType Overlord::IsSingleColorStack(const std::vector<Object> &in)
{
	assert(in.size() > 0);
	if (!in[0].IsCake())
	{
		return ObjectType::Unknown;
	}
	ObjectType color = in[0].Type;
	for (int i = 1; i < in.size(); i++)
	{
		if (in[i].Type != color)
		{
			return ObjectType::Unknown;
		}
	}
	return color;
}

vector<vector<Object>> Overlord::FindCakeStacks(const std::vector<Object> &in)
{
	vector<vector<Object>> outvec;
	vector<bool> observed(in.size(), false);
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
		vector<Object> stack;
		stack.push_back(cake0);
		for (int j = i+1; j < in.size(); j++)
		{
			auto& caken = in[j];
			if (observed[j])
			{
				continue;
			}
			if (!caken.IsCake())
			{
				//not cake
				continue;
			}
			if ((caken.position-cake0.position).length() < CakeTolerance) //add tolerance here
			{
				numcakes++;
				mean += caken.position;
				observed[j] = true;
				stack.push_back(caken);
			}
			
		}
		outvec.push_back(stack);
	}
	return outvec;
}

void Overlord::FilterType(std::vector<Object> &in, const set<ObjectType> allowed)
{
	in.erase(std::remove_if(in.begin(), in.end(), [&allowed](Object o)
	{
		return allowed.find(o.Type) == allowed.end();
	}), in.end());
}

void Overlord::DistanceSort(std::vector<Object> &in, Vector2dd reference)
{
	sort(in.begin(), in.end(), [&reference](Object a, Object b)
	{
		return (reference-a.position).lengthsquared()<(reference-b.position).lengthsquared();
	});
}

void Overlord::DistanceClip(std::vector<Object> &in, Vector2dd reference, double MaxDistance)
{
	double d2 = MaxDistance*MaxDistance;
	in.erase(std::remove_if(in.begin(), in.end(), [&reference, d2](Object o)
	{
		return (o.position-reference).lengthsquared() > d2;
	}), in.end());
}

std::set<ObjectType> Overlord::GetCakeComplement(const std::set<ObjectType> &In)
{
	set<ObjectType> AllColors = {ObjectType::CakeBrown, ObjectType::CakeYellow, ObjectType::CakePink}, ColorsNeeded;
	for (auto color : AllColors) //ColorsNeeded = AllColors - ColorsGot
	{
		if (In.find(color) == In.end())
		{
			ColorsNeeded.insert(color);
		}
	}
	return ColorsNeeded;
}