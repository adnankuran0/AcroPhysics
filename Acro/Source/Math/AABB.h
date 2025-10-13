#ifndef ACRO_AABB_H
#define ACRO_AABB_H

#include "Math/Vector3.h"


namespace Acro::Math {

class AABB
{
public:
	Acro::Math::Vector3 min;
	Acro::Math::Vector3 max;
	float padding;

	AABB() : min(std::numeric_limits<float>::max()), max(std::numeric_limits<float>::max()) {}
	AABB(const Acro::Math::Vector3& min, const Acro::Math::Vector3& max) : min(min), max(max) {}

	static AABB FromCenterAndExtent(const Acro::Math::Vector3& center, const Acro::Math::Vector3& extent)
	{
		AABB box;
		box.min = center - extent;
		box.max = center + extent;
		return box;
	}

	static AABB FromVertices(const Acro::Math::Vector3* vertices, size_t count, float padding = 0.05f)
	{
		Acro::Math::Vector3 minimum = vertices[0];
		Acro::Math::Vector3 maximum = vertices[0];
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

	Acro::Math::Vector3 Size()
	{
		return max - min;
	}

	bool Intersects(const AABB& other) const
	{
		return (min.x <= other.max.x && max.x >= other.min.x) &&
			(min.y <= other.max.y && max.y >= other.min.y) &&
			(min.z <= other.max.z && max.z >= other.min.z);
	}

	bool Conains(const Acro::Math::Vector3& point)
	{
		return (point.x >= min.x && point.x <= max.x) &&
			(point.y >= min.y && point.y <= max.y) &&
			(point.z >= min.z && point.z <= max.z);
	}

	void Translate(const Acro::Math::Vector3& offset)
	{
		min += offset;
		max += offset;
	}

	void Reset()
	{
		min = Acro::Math::Vector3(std::numeric_limits<float>::max());
		max = Acro::Math::Vector3(std::numeric_limits<float>::max());
	}

};

}

#endif // !ACRO_AABB_H
