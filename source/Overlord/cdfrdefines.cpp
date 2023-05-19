#include "Overlord/cdfrdefines.hpp"

const double Overlord::CakeTolerance= 0.04, 
Overlord::PositionTolerance = 0.01, 
Overlord::AngleTolerance = 5/180.0*M_PI,
Overlord::CakeHeight = 0.02,
Overlord::CakeRadius = 0.06,
Overlord::ZoneWidth = 0.45;

Overlord::monotime Overlord::GetNow()
{
	return monoclock::now();
}

Overlord::monodelta Overlord::GetDeltaFromNow(Overlord::monotime then)
{
	return GetNow()-then;
}