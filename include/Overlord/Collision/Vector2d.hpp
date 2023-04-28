#pragma once

#include <cmath>

namespace Overlord
{
	template<class T>
	struct Vector2d
	{
		T x=0,y=0;

		Vector2d()
		:x(0), y(0)
		{}

		Vector2d(T InX, T InY)
		:x(InX), y(InY)
		{}

		Vector2d(T rotation)
		{
			sincos(rotation, &y, &x);
		}

		T lengthsquared() const
		{
			return x*x+y*y;
		}

		T length() const
		{
			return sqrt(lengthsquared());
		}

		T angle() const
		{
			return atan2(y, x);
		}

		Vector2d<T> rotate(T angle) const
		{
			T sv, cv;
			sincos(angle, &sv, &cv);
			return {x*cv-y*sv, x*sv+y*cv};
		}

		void normalize()
		{
			T len = length();
			if (len < __DBL_EPSILON__)
			{
				x=0;
				y=0;
				return;
			}
			*this /= len;
		}
		Vector2d<T> normalized() const
		{
			T len = length();
			if (len < __DBL_EPSILON__)
			{
				return Vector2d<T>(0,0);
			}
			return (*this)/length();
		}

		Vector2d<T> dot(Vector2d<T> &other) const
		{
			return Vector2d<T>(x*other.x, y*other.y);
		}

		T sum()
		{
			return x+y;
		}

		void operator+=(const Vector2d<T>& other)
		{
			x+=other.x;
			y+=other.y;
		}
		Vector2d<T> operator+(const Vector2d<T> &other) const
		{
			return Vector2d<T>(x+other.x, y+other.y);
		}
		void operator-=(const Vector2d<T>& other)
		{
			x-=other.x;
			y-=other.y;
		}
		Vector2d<T> operator-(const Vector2d<T> &other) const
		{
			return Vector2d<T>(x-other.x, y-other.y);
		}
		void operator*=(const Vector2d<T>& other)
		{
			x*=other.x;
			y*=other.y;
		}
		Vector2d<T> operator*(const Vector2d<T> &other) const
		{
			return Vector2d<T>(x*other.x, y*other.y);
		}
		void operator/=(const Vector2d<T>& other)
		{
			x/=other.x;
			y/=other.y;
		}
		Vector2d<T> operator/(const Vector2d &other) const
		{
			return Vector2d<T>(x/other.x, y/other.y);
		}
		void operator*=(T value)
		{
			x*=value;
			y*=value;
		}
		Vector2d<T> operator*(T value) const
		{
			return Vector2d<T>(x*value, y*value);
		}
		void operator/=(T value)
		{
			x/=value;
			y/=value;
		}
		Vector2d<T> operator/(T value) const
		{
			return Vector2d<T>(x/value, y/value);
		}
	};
	
} // namespace Overlord
