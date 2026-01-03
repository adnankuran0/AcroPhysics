#include <algorithm>
#include "Math/AABB.h"

using namespace Acro::Math;

AABB AABB::FromVertices(const Vector3* vertices, size_t count, float padding) noexcept
{
	Vector3 minimum = vertices[0];
	Vector3 maximum = vertices[0];
	for (size_t i = 1; i < count; ++i)
	{
		minimum.x = std::min(minimum.x, vertices[i].x);
		minimum.y = std::min(minimum.y, vertices[i].y);
		minimum.z = std::min(minimum.z, vertices[i].z);

		maximum.x = std::max(maximum.x, vertices[i].x);
		maximum.y = std::max(maximum.y, vertices[i].y);
		maximum.z = std::max(maximum.z, vertices[i].z);
	}

	minimum -= padding;
	maximum += padding;

	AABB box(minimum, maximum);
	box.padding = padding;
	return box;
}