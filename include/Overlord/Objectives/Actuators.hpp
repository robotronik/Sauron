#pragma once

namespace Overlord
{
	enum class ActuatorType
	{
		Wheels,
		Claws,
		Tray0,
		Tray1,
		Tray2
	};

	ActuatorType GetTrayType(int index);
} // namespace Overlord
