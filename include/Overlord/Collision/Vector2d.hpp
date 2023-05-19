#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <algorithm>
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

		Vector2d<T> abs()
		{
			return Vector2d<T>(std::abs(x), std::abs(y));
		}

		T dot(const Vector2d<T> &other) const
		{
			return x*other.x + y*other.y;
		}

		T sum()
		{
			return x+y;
		}

		T max()
		{
			return std::max(x,y);
		}

		Vector2d<T> min(const Vector2d<T>& other)
		{
			return Vector2d<T>(std::min<T>(x,other.x), std::min<T>(y, other.y));
		}

		Vector2d<T> max(const Vector2d<T>& other)
		{
			return Vector2d<T>(std::max<T>(x,other.x), std::max<T>(y, other.y));
		}

		std::string ToString() const
		{
			std::ostringstream oss;
			oss << "(" << x << ", " << y << ")";
			return oss.str();
		}

		bool operator<(const Vector2d<T>& other)
		{
			return x<other.x && y<other.y;
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

		Vector2d<T> operator-() const
		{
			return Vector2d<T>(-x, -y);
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

		bool operator==(const Vector2d<T> &other)
		{
			return other.x==x && other.y==y;
		}

		bool operator!=(const Vector2d<T> &other)
		{
			return other.x!=x || other.y!=y;
		}

		Vector2d<T> operator%(const Vector2d<T> &other) const
		{
			return Vector2d<T>(fmod(x, other.x), fmod(y, other.y));
		}


	};

	typedef Vector2d<double> Vector2dd;

	template<class T>
	T wraptwopi(T in)
	{
		T rem = fmod(in, M_PI*2);
		if (rem < -M_PI)
		{
			return rem + M_PI*2;
		}
		else if (rem > M_PI)
		{
			return rem - M_PI*2;
		}
		return rem;
	}
	
} // namespace Overlord
