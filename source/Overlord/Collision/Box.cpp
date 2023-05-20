/* 

	Système de détection entre boites.

*/

#include "Overlord/Collision/Box.hpp"

using namespace Overlord;
using namespace std;


// Obtenir les coordonnées des coins de la boite.  Les coins sont stockés dans un vecteur. La méthode utilise des vecteurs pour décrire la rotation de la boite.
std::vector<Vector2dd> Box::GetCorners() const
{
	Vector2dd xvec(rot), yvec(rot+M_PI_2);
	vector<Vector2dd> corners(4, Vector2dd(0,0));
	for (int i = 0; i < 4; i++)
	{
		double xmir = i & 1 ? -1:1;
		double ymir = i & 2 ? -1:1;
		corners[i] = origin + xvec*extents.x + yvec*extents.y;
	}
	return corners;
}

// Cette méthode vérifie si la boite actuelle est en collision avec une autre boite.
// Pour ce faire, on récupère les coins de la boite et on vérifie si un des coins est dans la boite.
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