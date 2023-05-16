#include "Overlord/Collision/Pathfinder.hpp"

#include <set>
#include <utility>
#include <algorithm>
#include <assert.h>

using namespace Overlord;
using namespace std;

bool Pathfinder::IsInArena(Vector2dd pos) const
{
	return pos.abs() < ArenaSizeRemoved;
}

void Pathfinder::SetArenaSize(Vector2dd extent, double RobotRadius)
{
	ArenaSizeRemoved = extent-Vector2dd(RobotRadius, RobotRadius);
}

bool Pathfinder::IsColliding(Vector2dd pos) const
{
	for (int i = 0; i < Obstacles.size(); i++)
	{
		if ((pos-Obstacles[i].position).length() < Obstacles[i].radius)
		{
			return true;
		}
	}
	return false;
}

void Pathfinder::SetObstacles(std::vector<Obstacle> InObstacles)
{
	Obstacles = InObstacles;
	Clumps.clear();
}

void Pathfinder::AddCherryHolders(double RobotRadius)
{
	static const double CherryHolderLength = 0.270;
	static const double CherryHolderWidth = 0.030;
	double obsradius = /*CherryHolderWidth/2+*/RobotRadius;
	Obstacles.reserve(Obstacles.size()+3*4);
	for (int i = -5; i < 6; i+=5)
	{
		Obstacle xaligned;
		xaligned.position.x = i*CherryHolderLength/10.0;
		xaligned.position.y = 1-CherryHolderWidth/2;
		xaligned.radius = obsradius;
		xaligned.ClumpIdx = -1;
		Obstacles.push_back(xaligned);
		xaligned.position.y *= -1;
		Obstacles.push_back(xaligned);
	}
	for (int i = 0; i < 10; i+=3)
	{
		Obstacle yaligned;
		yaligned.position.y = 0;
		yaligned.position.x = -1.5+CherryHolderWidth/2+i*CherryHolderLength/9;
		yaligned.radius = obsradius;
		yaligned.ClumpIdx = -1;
		Obstacles.push_back(yaligned);
		yaligned.position.x *= -1;
		Obstacles.push_back(yaligned);
	}
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

std::vector<Vector2dd> Pathfinder::GetClumpEdges(int clumpidx)
{
	Vector2dd bbmax(-INFINITY, -INFINITY), bbmin(INFINITY, INFINITY);
	assert(clumpidx >= 0 && clumpidx < Clumps.size());
	auto& clump = Clumps[clumpidx];
	assert(clump.size() > 0);
	for (int i = 0; i < clump.size(); i++)
	{
		auto& obstacle = clump[i];
		Vector2dd obmin = obstacle.position - Vector2dd(obstacle.radius, obstacle.radius);
		Vector2dd obmax = obstacle.position + Vector2dd(obstacle.radius, obstacle.radius);
		bbmax = bbmax.max(obmax);
		bbmin = bbmin.min(obmin);
	}
	bbmax += Vector2dd(2e-3, 2e-3);
	bbmin -= Vector2dd(2e-3, 2e-3);
	vector<Vector2dd> waypoints = {bbmin, bbmax, {bbmin.x, bbmax.y}, {bbmax.x, bbmin.y}};
	for (int i = waypoints.size() - 1; i >= 0; i--)
	{
		if (!IsInArena(waypoints[i]))
		{
			goto deletethis;
		}
		if (IsColliding(waypoints[i]))
		{
			goto deletethis;
		}
		continue;
	deletethis:
		waypoints.erase(waypoints.begin()+i);
	}
	return waypoints;
}

double Pathfinder::GetPathLength(const Path& inpath)
{
	double length = 0;
	for (int i = 0; i < inpath.size()-1; i++)
	{
		length += (inpath[i]-inpath[i+1]).length();
	}
	return length;
}

void Pathfinder::CombinePaths(Path& receiver, const Path& donor)
{
	assert((receiver[receiver.size()-1]-donor[0]).length() < 1e-3);
	int offset = receiver.size()-1;
	receiver.resize(offset+donor.size());
	for (int i = 0; i < donor.size(); i++)
	{
		receiver[i+offset] = donor[i];
	}
}

optional<Path> Pathfinder::Pathfind(Vector2dd start, Vector2dd end, int depth)
{
	if (depth <= 0)
	{
		return nullopt;
	}
	
	ComputeClumps();
	if (!IsInArena(start))
	{
		return nullopt;
	}
	if (!IsInArena(end))
	{
		return nullopt;
	}
	
	
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
	sort(hitmap.begin(), hitmap.end(), [](const pair<int, double> & A, const pair<int, double> & B){
		return A.second < B.second;
	});
	int obstacleidx = hitmap[0].first;
	int clumpidx = Obstacles[obstacleidx].ClumpIdx;
	auto edges = GetClumpEdges(clumpidx);
	if (edges.size() == 0)
	{
		return nullopt;
	}
	optional<Path> bestpath = nullopt;
	double bestpathlength = INFINITY;
	for (auto & edge : edges)
	{
		auto toedge = Pathfind(start, edge, depth-1);
		if (!toedge.has_value())
		{
			continue;
		}
		auto fromedge = Pathfind(edge, end, depth-1);
		if (!fromedge.has_value())
		{
			continue;
		}
		CombinePaths(toedge.value(), fromedge.value());
		double pathlength = GetPathLength(toedge.value());
		if (pathlength < bestpathlength)
		{
			bestpath = toedge;
			bestpathlength = pathlength;
		}
	}
	return bestpath;
}