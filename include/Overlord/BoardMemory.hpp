#pragma once

#include <vector>
#include <cstdint>
#include <optional>
#include "Overlord/Collision/Vector2d.hpp"

#ifdef WITH_SAURON
#include "visualisation/BoardGL.hpp"
#endif

namespace Overlord{

	enum class ObjectType : unsigned int
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
		Vector2dd position;
		double Rot;

		Object(ObjectType type = ObjectType::Unknown, Vector2dd InPos = {0,0}, double rot=0)
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

		bool IsCake() const
		{
			return Type == ObjectType::CakeBrown || Type == ObjectType::CakePink || Type == ObjectType::CakeYellow;
		}

	};

	class RobotHAL;
	class RobotMemory
	{
	public:
		std::vector<Object> CakeTrays[4]; //Tray 0 is the claws
		std::vector<Object> Cherries;

		void CopyFrom(const RobotMemory& other)
		{
			for (int i = 0; i < sizeof(CakeTrays)/sizeof(CakeTrays[0]); i++)
			{
				CakeTrays[i] = other.CakeTrays[i];
			}
			Cherries = other.Cherries;
		}

		//take cakes on the terrain into the claws
		int ClawCake(RobotHAL* robot, std::vector<Object> &BoardState, bool take);

		//push cakes from the claws to a tray or back
		bool TransferCake(RobotHAL* robot, int trayidx, bool ToTray);
	};

	std::vector<Object> FindObjects(const std::vector<Object> &in, unsigned int TypeFilter);

	std::vector<Object> FindObjectsSorted(const std::vector<Object> &in, unsigned int TypeFilter, Vector2dd SearchPos);

	std::optional<Vector2dd> FindNearestCakeStack(const std::vector<Object> &in, Vector2dd SearchPos);
}
