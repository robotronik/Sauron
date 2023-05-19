#pragma once

#include <chrono>
#include <cmath>

namespace Overlord
{
	extern const double CakeTolerance, PositionTolerance, AngleTolerance, CakeHeight, CakeRadius, ZoneWidth;

	typedef std::chrono::steady_clock monoclock;

	typedef std::chrono::time_point<monoclock> monotime;
	typedef std::chrono::duration<double> monodelta;

	monotime GetNow();

	monodelta GetDeltaFromNow(monotime then);
};