#ifndef ACRO_SHAPE_DATA_H
#define ACRO_SHAPE_DATA_H

#include <vector>
#include "Math/Vector3.h"

namespace Acro::Core {

enum class ShapeType
{
	Null,
	Box,
	Sphere
};

struct ShapeData
{
	std::vector<ShapeType> shapeTypes;
	std::vector<Acro::Math::Vector3> offsets;
	std::vector<Acro::Math::Vector3> extents; // Box shape
	std::vector<float> radii; // Sphere shape
	std::vector<uint8_t> dirtyFlags;
};
}

#endif // !ACRO_SHAPE_DATA_H
