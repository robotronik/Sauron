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
	public:
		void SetObstacles(std::vector<Obstacle> InObstacles);

		void ComputeClumps();

		std::optional<Path> Pathfind(Vector2dd start, Vector2dd end);
	};
} // namespace Overlord

