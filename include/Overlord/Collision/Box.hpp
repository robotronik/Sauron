#pragma once

#include <vector>
#include "Overlord/Collision/Vector2d.hpp"

namespace Overlord
{
	struct Box
	{
		Vector2d<double> origin, extents;
		double rot=0;

		Box(Vector2d<double> InExtents = {1,1}, Vector2d<double> InOrigin = {0,0}, double InRot = 0.0)
		:origin(InOrigin), extents(InExtents), rot(InRot)
		{};

		~Box();

		std::vector<Vector2d<double>> GetCorners() const;

		//should be checked both ways
		bool Colliding(const Box &other) const;
	};
	
} // namespace Overlord
