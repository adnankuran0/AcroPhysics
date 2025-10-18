#ifndef ACRO_COLLISION_LAYER_H
#define ACRO_COLLISION_LAYER_H

#include <cstdint>
#include <initializer_list>

namespace Acro {

using LayerMask = uint32_t;

// supports 1-32 masks and layers

struct CollisionFilter
{
	LayerMask layers = 0;
	LayerMask mask = 0xFFFFFFFFu;
};

class CollisonLayer
{
public:
	inline static constexpr LayerMask LayerFromIndex(uint32_t index) noexcept
	{
		return 1u << (index - 1); 
	}

	inline static constexpr LayerMask MakeMask(std::initializer_list<uint32_t> indices) noexcept
	{
		LayerMask m = 0;
		for (uint32_t i : indices)
			m |= LayerFromIndex(i);
		return m;
	}

	inline static void AddLayer(LayerMask& mask, uint32_t index) noexcept
	{
		mask |= LayerFromIndex(index);
	}

	inline static void RemoveLayer(LayerMask& mask, uint32_t index) noexcept
	{
		mask &= ~LayerFromIndex(index);
	}

	inline static bool ShouldCollide(const CollisionFilter& a, const CollisionFilter& b)
	{
		return ((a.mask & b.layers) != 0) && ((b.mask & a.layers) != 0);
	}
};




}


#endif // !ACRO_COLLISION_LAYER_H
