#pragma once

#include <vector>
#include "Overlord/Collision/Vector2d.hpp"

namespace Overlord
{
	struct Box
	{
		Vector2dd origin, extents;
		double rot=0;

		Box(Vector2dd InExtents = {1,1}, Vector2dd InOrigin = {0,0}, double InRot = 0.0)
		:origin(InOrigin), extents(InExtents), rot(InRot)
		{};

		~Box();

		std::vector<Vector2dd> GetCorners() const;

		//should be checked both ways
		bool Colliding(const Box &other) const;
	};
	
} // namespace Overlord
