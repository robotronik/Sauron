#include "Overlord/Objectives/Actuators.hpp"

#include <cassert>

Overlord::ActuatorType Overlord::GetTrayType(int index)
{
	switch (index)
	{
	case 0:
		return ActuatorType::Tray0;
	case 1:
		return ActuatorType::Tray1;
	case 2:
		return ActuatorType::Tray2;
	
	default:
		assert(0);
		return ActuatorType::Wheels;
	}
}