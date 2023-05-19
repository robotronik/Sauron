#pragma once

#include <vector>
#include <optional>
#include "Overlord/Collision/Vector2d.hpp"

namespace Overlord
{
	
	struct Obstacle
	{
		Vector2dd position = {0,0};
		double radius = 1;
		int ClumpIdx = -1;
	};

	typedef std::vector<Vector2dd> Path;
	

	class Pathfinder
	{
		std::vector<Obstacle> Obstacles;
		std::vector<std::vector<Obstacle>> Clumps;
		Vector2dd ArenaSizeRemoved;
	public:
		Vector2dd RobotHalfExtent;

		bool IsInArena(Vector2dd pos) const;

		Vector2dd ClampToArena(Vector2dd pos) const;

		void SetArenaSize(Vector2dd extent, double RobotRadius);

		static Vector2dd Repulse(Vector2dd pos, Vector2dd repulsor, double radius);

		bool IsColliding(Vector2dd pos) const;

		static Vector2dd ProjectOnLine(Vector2dd X, Vector2dd A, Vector2dd B);

		void SetObstacles(std::vector<Obstacle> InObstacles);

		void AddCherryHolders(double RobotRadius);

		void ComputeClumps();

		std::vector<Vector2dd> GetClumpEdges(int clumpidx);

		static void SortEdgesPath(std::vector<Vector2dd> &edges, Vector2dd start, Vector2dd end);

		static double GetPathLength(const Path& inpath);

		static void CombinePaths(Path& receiver, const Path& donor);

		static void RemovePathDuplicates(Path& path);

		std::vector<std::pair<int, double>> GetIntersections(Vector2dd start, Vector2dd end);

		std::optional<Path> Pathfind(Vector2dd start, Vector2dd end, int depth = 5);

		bool IsOffsetInSingularity(Vector2dd position, Vector2dd target, Vector2dd offset);

		//distance, angle
		std::vector<std::pair<double, double>> GetOffsetTarget(Vector2dd position, Vector2dd target, Vector2dd offset);

		std::optional<Path> PathfindOffset(Vector2dd start, Vector2dd end, Vector2dd offset, int depth = 5);
	};
} // namespace Overlord

