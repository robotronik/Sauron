#include "Overlord/Collision/Pathfinder.hpp"

#include <set>
#include <utility>

using namespace Overlord;
using namespace std;


void Pathfinder::SetObstacles(std::vector<Obstacle> InObstacles)
{
	Obstacles = InObstacles;
	Clumps.clear();
}

void Pathfinder::ComputeClumps()
{
	if (Clumps.size() != 0)
	{
		return;
	}
	
	for (int i = 0; i < Obstacles.size(); i++)
	{
		int Clumpidx = Clumps.size();
		for (int j = 0; j < i; j++)
		{
			if ((Obstacles[i].position-Obstacles[j].position).length()<(Obstacles[i].radius+Obstacles[j].radius) )
			{
				Clumpidx = Obstacles[j].ClumpIdx;
				break;
			}
		}
		Obstacles[i].ClumpIdx = Clumpidx;
		if (Clumpidx == Clumps.size())
		{
			Clumps.push_back({Obstacles[i]});
		}
		else
		{
			Clumps[Clumpidx].push_back(Obstacles[i]);
		}
		
	}
	
}

optional<Path> Pathfinder::Pathfind(Vector2dd start, Vector2dd end)
{
	ComputeClumps();
	Path currpath = {start, end};
	auto dpos = end-start;
	auto direction = dpos.normalized();
	double dtt = dpos.length();
	set<int> HitClumps;
	vector<pair<int, double>> hitmap;
	for (int i = 0; i < Obstacles.size(); i++)
	{
		auto& obstacle = Obstacles[i];
		auto deltaPosObstacle = obstacle.position-start;
		auto projection = deltaPosObstacle.dot(direction);
		double HitDistance = max(min(projection, dtt), 0.0);
		auto projOnLine = direction * HitDistance;
		if ((deltaPosObstacle-projOnLine).length() < obstacle.radius)
		{
			HitClumps.emplace(obstacle.ClumpIdx);
			hitmap.push_back({i, HitDistance});
		}
	}
	if (hitmap.size() == 0)
	{
		return currpath;
	}
	else
	{
		return nullopt;
	}
	
	
}