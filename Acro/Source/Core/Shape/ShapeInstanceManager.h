#ifndef ACRO_SHAPE_INSTANCE_MANAGER_H
#define ACRO_SHAPE_INSTANCE_MANAGER_H

#include <cstdint>
#include "Core/Shape/ShapeInstanceData.h"

namespace Acro::Core {

struct ShapeInstanceHandle
{
	uint32_t index;
	uint32_t generation;
	
	static constexpr ShapeInstanceHandle Null() { return { std::numeric_limits<uint32_t>::max(),std::numeric_limits<uint32_t>::max() }; }

	bool operator==(const ShapeInstanceHandle& other) const noexcept
	{
		return index == other.index && generation == other.generation;
	}
};

class ShapeInstanceManager
{
public:
	ShapeInstanceHandle CreateShapeInstance(const ShapeHandle& shapeHandle, const BodyHandle& bodyHandle);
	void DestroyShapeInstance(const ShapeInstanceHandle& handle);

	inline bool IsValid(const ShapeInstanceHandle& handle) const noexcept
	{
		if (handle.index >= m_Generations.size()) return false;
		return m_Generations[handle.index] == handle.generation;
	}

	void UpdateWorldData(Acro::Core::BodyManager& bodyManager, Acro::Core::ShapeManager& shapeManager);
	

	[[nodiscard]] inline ShapeInstanceData& GetData() noexcept { return m_ShapeInstanceData; }

private:

	ShapeInstanceData m_ShapeInstanceData; // dense
	std::vector<uint32_t> m_Sparse;
	std::vector<uint32_t> m_DenseToHandle;
	std::vector<uint32_t> m_Generations;
	std::vector<ShapeInstanceHandle> m_FreeHandles;

};

}

#endif // !ACRO_SHAPE_INSTANCE_MANAGER_H
