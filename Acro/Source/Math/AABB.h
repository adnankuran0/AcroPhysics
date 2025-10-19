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
	AABB(const Acro::Math::Vector3& min, const Acro::Math::Vector3& max,float padding = 0.02f) : min(min - padding), max(max + padding) {}

	inline static AABB FromCenterAndExtent(const Acro::Math::Vector3& center, const Acro::Math::Vector3& extent,float padding = 0.02f) noexcept
	{
		AABB box;
		box.min = center - extent - padding;
		box.max = center + extent + padding;
		return box;
	}

	static AABB FromVertices(const Acro::Math::Vector3* vertices, size_t count, float padding = 0.02f) noexcept;

	inline Acro::Math::Vector3 Size() const noexcept
	{
		return max - min;
	}

	inline bool Intersects(const AABB& other) const noexcept
	{
		return (min.x <= other.max.x && max.x >= other.min.x) &&
			(min.y <= other.max.y && max.y >= other.min.y) &&
			(min.z <= other.max.z && max.z >= other.min.z);
	}

	inline bool Conains(const Acro::Math::Vector3& point) const noexcept
	{
		return (point.x >= min.x && point.x <= max.x) &&
			(point.y >= min.y && point.y <= max.y) &&
			(point.z >= min.z && point.z <= max.z);
	}

	inline void Translate(const Acro::Math::Vector3& offset) noexcept
	{
		min += offset;
		max += offset;
	}

	inline void Reset() noexcept
	{
		min = Acro::Math::Vector3(std::numeric_limits<float>::max());
		max = Acro::Math::Vector3(std::numeric_limits<float>::max());
	}

};

}

#endif // !ACRO_AABB_H
