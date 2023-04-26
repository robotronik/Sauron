#pragma once

#include <cmath>

namespace Overlord
{
	template<class T>
	struct Matrix22
	{
		double values[2][2] = {0}; //column then line

		Matrix22<T> Rotation(T angle)
		{
			T sv, cv;
			sincos(angle, sv, cv);
			
		}
	};
	
} // namespace Overlord
