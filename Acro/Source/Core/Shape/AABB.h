#ifndef ACRO_AABB_H
#define ACRO_AABB_H

#include "Math/Vector3.h"


namespace Acro::Core {

class AABB
{
public:
	Acro::Math::Vector3 min;
	Acro::Math::Vector3 max;

	AABB() : min(std::numeric_limits<float>::max()), max(std::numeric_limits<float>::max()) {}
	AABB(const Acro::Math::Vector3& min, const Acro::Math::Vector3& max) : min(min), max(max) {}

	static AABB FromCenterAndExtent(const Acro::Math::Vector3& center, const Acro::Math::Vector3& extent)
	{
		AABB box;
		box.min = center - extent;
		box.max = center + extent;
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
