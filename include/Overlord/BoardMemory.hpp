#pragma once

#include <vector>
#include <cstdint>
#include "Overlord/Collision/Vector2d.hpp"

#ifdef WITH_SAURON
#include "visualisation/BoardGL.hpp"
#endif

namespace Overlord{

	enum class ObjectType : std::uint32_t
	{
		Unknown 	= 0,
		Robot 		= 0b1,
		Cherry 		= 0b10,
		CakeBrown 	= 0b100,
		CakeYellow 	= 0b1000,
		CakePink 	= 0b10000,
		BlueDropZone	= 0b100000,
		GreenDropZone	= 0b1000000
	};

	struct Object
	{
		ObjectType Type;
		Vector2d<double> position;
		double Rot;

		Object(ObjectType type = ObjectType::Unknown, Vector2d<double> InPos = {0,0}, double rot=0)
		:Type(type), position(InPos), Rot(rot)
		{};

		const bool operator==(const Object& other)
		{
			if (other.Type != Type)
			{
				return false;
			}
			if (abs(position.x-other.position.x) > __DBL_EPSILON__)
			{
				return false;
			}
			if (abs(position.y-other.position.y) > __DBL_EPSILON__)
			{
				return false;
			}
			return true;
		}

	};

	class BoardMemory
	{
	public:
		std::vector<Object> ObjectsOnBoard;

		std::vector<Object> FindObjects(std::uint32_t TypeFilter);

		std::vector<Object> FindObjectsSorted(std::uint32_t TypeFilter, Vector2d<double> SearchPos);
	};

	class RobotMemory
	{
	public:
		std::vector<Object> CakeTrays[4]; //Tray 0 is the claws
		std::vector<Object> Cherries;
	};
}
