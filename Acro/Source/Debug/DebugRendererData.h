#ifndef ACRO_DEBUG_RENDERER_DATA_H
#define ACRO_DEBUG_RENDERER_DATA_H

#include <vector>
#include "Math/Vector3.h"

namespace Acro::Debug {

struct DebugVertex
{
	Acro::Math::Vector3 position;
	Acro::Math::Vector3 color;
};

enum class DebugPrimitiveType
{
	Line,
	Triangle,
	Point
};

struct DebugPrimitive
{
	DebugPrimitiveType type;
	std::vector<DebugVertex> vertices;
	float duration;
};

}

#endif // !ACRO_DEBUG_RENDERER_DATA_H
