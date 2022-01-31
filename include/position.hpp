#pragma once
#include <opencv2/core.hpp>
#include <math.h>

template<class T>
struct FVector2D
{
	T x;
	T y;
	
	FVector2D()
		:x(0), y(0)
	{}

	FVector2D(T s)
	:x(s), y(s)
	{}

	FVector2D(T inX, T inY)
		:x(inX), y(inY)
	{}

	template<class T2>
	FVector2D(FVector2D<T2> copy)
		:x(copy.x),
		y(copy.y)
	{}

	FVector2D(cv::Point_<T> point)
		:x(point.x),
		y(point.y)
	{}

	FVector2D(cv::Size_<T> size)
		:x(size.width),
		y(size.height)
	{}

	template<class T2>
	FVector2D operator+(const FVector2D<T2> &other)
	{
		return FVector2D(x+other.x, y+other.y);
	}

	template<class T2>
	FVector2D operator-(const FVector2D<T2> &other)
	{
		return FVector2D(x-other.x, y-other.y);
	}
	template<class T2>
	FVector2D operator*(const FVector2D<T2> &other)
	{
		return FVector2D(x*other.x, y*other.y);
	}
	template<class T2>
	FVector2D operator/(const FVector2D<T2> &other)
	{
		return FVector2D(x/other.x, y/other.y);
	}

	template<class T2>
	FVector2D operator*(const T2 &other)
	{
		return FVector2D(x*other, y*other);
	}
	template<class T2>
	FVector2D operator/(const T2 &other)
	{
		return FVector2D(x/other, y/other);
	}

	template<class T2>
	float operator|(const FVector2D<T2> &other) //dot product
	{
		return x*other.x + y*other.y;
	}

	FVector2D<T> operator%(const FVector2D<T> &other)
	{
		return FVector2D(fmod(x,other.x), fmod(y,other.y));
	}

	float Length()
	{
		return sqrt(x*x + y*y);
	}

	float Angle()
	{
		return atan2(y, x);
	}

	FVector2D<T> Abs()
	{
		return FVector2D(x < 0 ? -x : x, y < 0 ? -y : y);
	}

	cv::Size_<T> ToSize()
	{
		return cv::Size_<T>(x,y);
	}

	operator cv::Point_<T>()
	{
		return cv::Point_<T>(x,y);
	}

	static FVector2D<T> Max(FVector2D<T> Vec1, FVector2D<T> Vec2)
	{
		return FVector2D<T>(
			Vec1.x > Vec2.x ? Vec1.x : Vec2.x,
			Vec1.y > Vec2.y ? Vec1.y : Vec2.y
		);
	}

	static FVector2D<T> Min(FVector2D<T> Vec1, FVector2D<T> Vec2)
	{
		return FVector2D<T>(
			Vec1.x < Vec2.x ? Vec1.x : Vec2.x,
			Vec1.y < Vec2.y ? Vec1.y : Vec2.y
		);
	}

	static FVector2D<T> Clamp(FVector2D<T> Vec, FVector2D<T> Minimum, FVector2D<T> Maximum)
	{
		return Min(Max(Vec, Minimum), Maximum);
	}

};