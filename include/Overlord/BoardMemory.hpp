#pragma once

#include <vector>


#ifdef WITH_SAURON
#include "visualisation/BoardGL.hpp"
#endif

namespace Overlord{

	enum class ObjectType
	{
		Unknown,
		Robot,
		Cake,
		Cherry
	};

	struct Object
	{
		ObjectType type;
		double PosX, posY, rot;

	};

	class BoardMemory
	{
	public:
		std::vector<Object> ObjectsOnBoard;
	};

	class RobotMemory
	{
	public:
		std::vector<Object> CakeTrays[4]; //Tray 0 is the claws
		std::vector<Object> Cherries;
	};
}
