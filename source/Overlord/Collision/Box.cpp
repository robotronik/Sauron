#include "Overlord/Collision/Box.hpp"

using namespace Overlord;
using namespace std;

std::vector<Vector2d<double>> Box::GetCorners() const
{
	Vector2d<double> xvec(rot), yvec(rot+M_PI_2);
	vector<Vector2d<double>> corners(4, Vector2d<double>(0,0));
	for (int i = 0; i < 4; i++)
	{
		double xmir = i & 1 ? -1:1;
		double ymir = i & 2 ? -1:1;
		corners[i] = origin + xvec*extents.x + yvec*extents.y;
	}
	return corners;
}

bool Box::Colliding(const Box &other) const
{
	auto othercorners = other.GetCorners();
	double cv, sv;
	sincos(-rot, &sv, &cv);
	for (int i = 0; i < othercorners.size(); i++)
	{
		auto relpos = othercorners[i] - origin;
		Vector2d unrotated(relpos.x*cv - relpos.y*sv, relpos.x*sv + relpos.y*cv);
		if (unrotated.x < extents.x && unrotated.y < extents.y)
		{
			return true;
		}
		
	}
	return false;
}