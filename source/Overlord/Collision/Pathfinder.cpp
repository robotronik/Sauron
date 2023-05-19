#include "Overlord/Collision/Pathfinder.hpp"

#include <iostream>
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

Vector2dd Pathfinder::ClampToArena(Vector2dd pos) const
{
	if (abs(pos.x) > ArenaSizeRemoved.x)
	{
		pos.x = copysign(ArenaSizeRemoved.x, pos.x);
	}
	if (abs(pos.y) > ArenaSizeRemoved.y)
	{
		pos.y = copysign(ArenaSizeRemoved.y, pos.y);
	}
	return pos;
}

void Pathfinder::SetArenaSize(Vector2dd extent, double RobotRadius)
{
	ArenaSizeRemoved = extent-Vector2dd(RobotRadius, RobotRadius);
}

Vector2dd Pathfinder::Repulse(Vector2dd pos, Vector2dd repulsor, double radius)
{
	auto dpos = pos-repulsor;
	if (dpos.length() < radius)
	{
		dpos.normalize();
		return repulsor+dpos*radius;
	}
	return pos;
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

Vector2dd Pathfinder::ProjectOnLine(Vector2dd X, Vector2dd A, Vector2dd B)
{
	const Vector2dd AtoB = B-A;
	const Vector2dd dir = AtoB.normalized();
	const Vector2dd AtoX = X-A;
	const double projected = AtoX.dot(dir);
	const double clampeddist = max(min(projected, AtoB.length()), 0.0);
	return A + dir * clampeddist;
}

void Pathfinder::SetObstacles(std::vector<Obstacle> InObstacles)
{
	Obstacles.clear();
	Obstacles.reserve(InObstacles.size());
	//remove duplicates and obstacles included in others
	for (int i = 0; i < InObstacles.size(); i++)
	{
		auto& nob = InObstacles[i];
		bool dontplace = false;
		for (int j = 0; j < Obstacles.size(); j++)
		{
			auto& pob = Obstacles[j];
			auto dpos = nob.position-pob.position;
			auto dr = nob.radius-pob.radius;
			auto dist = dpos.length();
			if (dist <= abs(dr) + __DBL_EPSILON__)
			{
				if (dr < 0) //new obstacle included in old obstacle
				{}
				else //old obstacle included in new obstacle, new obstacle takes the place of the old obstacle
				{
					pob = nob;
				}
				dontplace = true;
				break;
			}
		}
		if (!dontplace)
		{
			Obstacles.push_back(nob);
		}
	}
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
			if ((Obstacles[i].position-Obstacles[j].position).length()<(Obstacles[i].radius+Obstacles[j].radius + RobotHalfExtent.y*2) )
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

vector<Vector2dd> Pathfinder::GetClumpEdges(int clumpidx)
{
	Vector2dd bbmax(-INFINITY, -INFINITY), bbmin(INFINITY, INFINITY);
	assert(clumpidx >= 0 && clumpidx < Clumps.size());
	auto& clump = Clumps[clumpidx];
	assert(clump.size() > 0);
	for (int i = 0; i < clump.size(); i++)
	{
		auto& obstacle = clump[i];
		const double avoidanceradius = obstacle.radius;
		const Vector2dd avoidvec(avoidanceradius, avoidanceradius);
		const Vector2dd obmin = obstacle.position - avoidvec;
		const Vector2dd obmax = obstacle.position + avoidvec;
		bbmax = bbmax.max(obmax);
		bbmin = bbmin.min(obmin);
	}
	const double extraoffset = RobotHalfExtent.y + 2e-3;
	const Vector2dd extravec = Vector2dd(extraoffset, extraoffset);
	bbmax += extravec;
	bbmin -= extravec;
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



void Pathfinder::SortEdgesPath(vector<Vector2dd> &edges, Vector2dd start, Vector2dd end)
{
	sort(edges.begin(), edges.end(), [&start, &end](Vector2dd& A, Vector2dd& B)
	{
		double Aexcursion = (A-ProjectOnLine(A, start, end)).lengthsquared();
		double Bexcursion = (B-ProjectOnLine(B, start, end)).lengthsquared();
		return Aexcursion < Bexcursion;
	});
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

void Pathfinder::RemovePathDuplicates(Path& path)
{
	Vector2dd lastpos = path[path.size()-1];
	for (int i = path.size() - 1; i >= 0; i--)
	{
		for (int j = 0; j < i; j++)
		{
			if ((path[i]-path[j]).length() < 1e-3)
			{
				path.erase(path.begin()+i);
				break;
			}
		}
	}
	path[path.size()-1] = lastpos;
}

vector<pair<int, double>> Pathfinder::GetIntersections(Vector2dd start, Vector2dd end)
{
	auto dpos = end-start;
	auto direction = dpos.normalized();
	double dtt = dpos.length();
	double dirangle = direction.angle();
	vector<pair<int, double>> hitmap;
	for (int i = 0; i < Obstacles.size(); i++)
	{
		auto& obstacle = Obstacles[i];
		const Vector2dd deltaPosObstacle = obstacle.position-start;
		const Vector2dd projOnLine = ProjectOnLine(obstacle.position, start, end);
		const Vector2dd projToObs = deltaPosObstacle-projOnLine;
		const double HitDistance = (projOnLine-start).length();
		const double leeway = projToObs.length();

		if (leeway < obstacle.radius)
		{
			hitmap.push_back({i, HitDistance});
			continue;
		}
		Vector2dd projnear = projToObs*(leeway - obstacle.radius)/leeway;
		Vector2dd localproj = projnear.rotate(-dirangle);
		if (localproj.abs() < RobotHalfExtent)
		{
			hitmap.push_back({i, HitDistance});
		}
		
	}
	sort(hitmap.begin(), hitmap.end(), [](const pair<int, double> & A, const pair<int, double> & B){
		return A.second < B.second;
	});
	return hitmap;
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
	const Vector2dd dpos = end-start;
	const Vector2dd direction = dpos.normalized();
	const double dtt = dpos.length();
	const set<int> HitClumps;
	const vector<pair<int, double>> hitmap = GetIntersections(start, end);
	if (hitmap.size() == 0)
	{
		return currpath;
	}
	int obstacleidx = hitmap[hitmap.size()/2].first;
	auto& obstacle = Obstacles[obstacleidx];
	if ((start-obstacle.position) < obstacle.radius+RobotHalfExtent.y && hitmap.size() == 1)
	{
		Vector2dd repulsed = Repulse(start, obstacle.position, obstacle.radius+RobotHalfExtent.y);
		currpath.insert(currpath.end(), repulsed);
		start = repulsed;
	}
	
	int clumpidx = Obstacles[obstacleidx].ClumpIdx;
	auto edges = GetClumpEdges(clumpidx);
	if (edges.size() == 0)
	{
		return nullopt;
	}
	SortEdgesPath(edges, start, end);
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
		if (currpath.size() >2)
		{
			for (int i = 1; i < currpath.size()-1; i++)
			{
				toedge.value().insert(toedge.value().begin()+i, currpath[i]);
			}
		}
		
		CombinePaths(toedge.value(), fromedge.value());
		double pathlength = GetPathLength(toedge.value());
		if (pathlength < bestpathlength)
		{
			bestpath = toedge;
			bestpathlength = pathlength;
			break;
		}
	}
	if (!bestpath.has_value())
	{
		return nullopt;
	}
	RemovePathDuplicates(bestpath.value());
	return bestpath;
}

bool Pathfinder::IsOffsetInSingularity(Vector2dd position, Vector2dd target, Vector2dd offset)
{
	auto deltapos = target-position;
	return deltapos.lengthsquared() < offset.lengthsquared() + __DBL_EPSILON__;
}

vector<pair<double, double>> Pathfinder::GetOffsetTarget(Vector2dd position, Vector2dd target, Vector2dd offset)
{
	auto deltapos = target-position;
	double angle = deltapos.angle();
	double dplen2 = deltapos.lengthsquared();
	double xsq = dplen2 - offset.y*offset.y;
	assert(xsq>=0);
	double x = sqrt(xsq);
	double offsetangle = asin(offset.y/sqrt(dplen2));
	
	
	vector<double> anglestotry = {angle-offsetangle, angle+offsetangle, angle-offsetangle+M_PI, angle+offsetangle+M_PI};
	vector<pair<double, double>> solutions;
	int correctangle = -1;
	//goto foundcorrect;
	for (int i = 0; i < anglestotry.size(); i++)
	{
		double thisangle = wraptwopi(anglestotry[i]); //thisangle will be the rotation we take
		Vector2dd offsetrotworld = position + offset.rotate(thisangle); //final pos of the offset
		Vector2dd deltalin = target-offsetrotworld; //final position of the robot : go forward the distance
		auto forward = Vector2dd(thisangle);
		auto side = Vector2d(thisangle+M_PI_2);
		double dist = deltalin.dot(forward);
		auto sideerror = deltalin.dot(side);
		if (abs(sideerror) < 1e-5)
		{
			solutions.push_back({dist, thisangle});
		}
	}
	return solutions;
}

optional<Path> Pathfinder::PathfindOffset(Vector2dd start, Vector2dd end, Vector2dd offset, int depth)
{
	Path path = {start, end};
	bool disablepathfinding = false;
	if (IsOffsetInSingularity(start, end, offset))
	{
		auto intermediate = start;
		double repulsionradius = offset.length()*1.2;
		for (int i = 0; i < 10; i++)
		{
			intermediate = Repulse(intermediate, end, repulsionradius);
			intermediate = ClampToArena(intermediate);
			if ((intermediate-end).length() > offset.length())
			{
				break;
			}
		}
		path[1] = intermediate;
		path.push_back(end);
		disablepathfinding = true;
	}
	auto offsetstart = path[path.size()-2];
	auto offsetend = path[path.size()-1];
	auto solutions = GetOffsetTarget(offsetstart, offsetend, offset);

	vector<Path> bestpath;
	vector<vector<pair<int, double>>> intersections;
	bool nointersections = disablepathfinding;
	for (int i = 0; i < solutions.size(); i++)
	{
		const double angle = solutions[i].second;
		const double distance = solutions[i].first;
		const Vector2dd targetpos = offsetstart+Vector2dd(angle)*distance;
		const Vector2dd endposoffset = targetpos+offset.rotate(angle);
		if (!IsInArena(targetpos))
		{
			continue;
		}
		
		auto isct = GetIntersections(offsetstart, targetpos);
		for (int i = isct.size() - 1; i >= 0; i--) //remove intersections emanating from target
		{
			auto& obstacle = Obstacles[isct[i].first];
			double enddist = (obstacle.position-targetpos).length();
			if (enddist < obstacle.radius)
			{
				//isct.erase(isct.begin()+i);
				//cout << "Cannot reach target : obstacle at target is too thick : " << obstacle.radius << " / " << enddist << endl;
				return nullopt;
			}
		}
		
		vector<Vector2dd> thispath = path;
		thispath[thispath.size()-1] = targetpos;
		if (isct.size() == 0)
		{
			if (!nointersections)
			{
				bestpath.clear();
				intersections.clear();
			}
			nointersections = true;
			bestpath.push_back(thispath);
			intersections.push_back(isct);
		}
		else if (!nointersections)
		{
			bestpath.push_back(thispath);
			intersections.push_back(isct);
		}
	}

	if (nointersections)
	{
		//return shortest path
		double bestdist = INFINITY;
		int bestidx = -1;
		for (int i = 0; i < bestpath.size(); i++)
		{
			double pl = GetPathLength(bestpath[i]);
			if (pl < bestdist)
			{
				bestdist = pl;
				bestidx = i;
			}
		}
		if (bestidx == -1)
		{
			return nullopt;
		}
		
		return bestpath[bestidx];
	}
	
	optional<Path> bestsinglepath = nullopt;
	double bestpathlength = INFINITY;
	for (int i = 0; i < bestpath.size(); i++)
	{
		int obstacleidx = intersections[i][0].first;
		int clumpidx = Obstacles[obstacleidx].ClumpIdx;
		auto edges = GetClumpEdges(clumpidx);
		SortEdgesPath(edges, start, end);
		for (auto& edge : edges)
		{
			auto toedge = Pathfind(start, edge, depth-1);
			if (!toedge.has_value())
			{
				continue;
			}
			auto fromedge = PathfindOffset(edge, end, offset, depth-1);
			if (!fromedge.has_value())
			{
				continue;
			}
			CombinePaths(toedge.value(), fromedge.value());
			double pathlength = GetPathLength(toedge.value());
			if (pathlength < bestpathlength)
			{
				bestsinglepath = toedge;
				bestpathlength = pathlength;
				break;
			}
		}
	}
	return bestsinglepath;
}