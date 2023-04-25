#pragma once

#include <vector>
#include <cstdint>

#ifdef WITH_SAURON
#include "visualisation/BoardGL.hpp"
#endif

namespace Overlord{

	enum class ObjectType : std::uint8_t
	{
		Unknown 	= 0,
		Robot 		= 0b1,
		Cherry 		= 0b10,
		CakeBrown 	= 0b100,
		CakeYellow 	= 0b1000,
		CakePink 	= 0b10000
	};

	struct Object
	{
		ObjectType Type;
		double PosX, PosY, Rot;

		Object(ObjectType type = ObjectType::Unknown, double posx=0, double posy=0, double rot=0)
		:Type(type), PosX(posx), PosY(posy), Rot(rot)
		{};

		const bool operator==(const Object& other)
		{
			if (other.Type != Type)
			{
				return false;
			}
			if (abs(PosX-other.PosX) > __DBL_EPSILON__)
			{
				return false;
			}
			if (abs(PosY-other.PosY) > __DBL_EPSILON__)
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

		std::vector<Object> FindObjects(std::uint8_t TypeFilter);

		std::vector<Object> FindObjectsSorted(std::uint8_t TypeFilter, double posX, double posY);
	};

	class RobotMemory
	{
	public:
		std::vector<Object> CakeTrays[4]; //Tray 0 is the claws
		std::vector<Object> Cherries;
	};
}
