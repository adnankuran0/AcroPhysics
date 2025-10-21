#ifndef ACRO_SHAPE_INSTANCE_MANAGER_H
#define ACRO_SHAPE_INSTANCE_MANAGER_H

#include <cstdint>
#include "Physics/Shape/ShapeInstanceData.h"
#include "Core/HandlePool.h"

namespace Acro::Physics {

class ShapeManager; // forward declaration

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

class ShapeInstanceManager : public Acro::Core::HandlePool<ShapeInstanceHandle>
{
public:
	ShapeInstanceHandle CreateShapeInstance(Acro::Physics::ShapeManager& shapeManager, const Acro::Physics::ShapeHandle& shapeHandle, const Acro::Physics::BodyHandle& bodyHandle);
	void DestroyShapeInstance(const ShapeInstanceHandle& handle);

	

	void UpdateWorldData(Acro::Physics::BodyManager& bodyManager, Acro::Physics::ShapeManager& shapeManager);
	
	inline Acro::Math::Vector3* GetWorldVertices(size_t denseIndex) noexcept
	{
		return m_ShapeInstanceData.worldVertexBuffer.data() + m_ShapeInstanceData.vertexOffsets[denseIndex];
	}
	inline size_t GetWorldVertexCount(size_t denseIndex) noexcept
	{
		return m_ShapeInstanceData.vertexCounts[denseIndex];
	}
	inline const Acro::Math::Matrix4& GetWorldTransform(const Acro::Physics::ShapeInstanceHandle& handle) const noexcept { return m_ShapeInstanceData.worldTransforms[m_Sparse[handle.index]]; }
	inline const Acro::Math::AABB& GetWorldAABB(const Acro::Physics::ShapeInstanceHandle& handle) const noexcept { return m_ShapeInstanceData.worldAABBs[m_Sparse[handle.index]]; }

	inline void SetCollisionFilter(const Acro::Physics::ShapeInstanceHandle& handle, const Acro::CollisionFilter& filter) { m_ShapeInstanceData.filters[m_Sparse[handle.index]] = filter; }
	inline CollisionFilter& GetCollisionFilter(const ShapeInstanceHandle& handle) noexcept { return m_ShapeInstanceData.filters[m_Sparse[handle.index]]; }

	inline void EnableLayer(const Acro::Physics::ShapeInstanceHandle& handle, uint32_t index) noexcept
	{
		CollisonLayer::AddLayer(GetCollisionFilter(handle).layers, index);
	}
	inline void DisableLayer(const Acro::Physics::ShapeInstanceHandle& handle, uint32_t index) noexcept
	{
		CollisonLayer::RemoveLayer(GetCollisionFilter(handle).layers, index);
	}
	inline void EnableMask(const Acro::Physics::ShapeInstanceHandle& handle, uint32_t index) noexcept
	{
		CollisonLayer::AddLayer(GetCollisionFilter(handle).mask, index);
	}
	inline void DisableMask(const Acro::Physics::ShapeInstanceHandle& handle, uint32_t index) noexcept
	{
		CollisonLayer::RemoveLayer(GetCollisionFilter(handle).mask, index);
	}

	[[nodiscard]] inline Acro::Physics::ShapeInstanceData& GetData() noexcept { return m_ShapeInstanceData; }

private:
	void SwapDenseData(size_t from, size_t to) noexcept;
	void PopBackDenseData(size_t vertexCount) noexcept;


	Acro::Physics::ShapeInstanceData m_ShapeInstanceData; // dense
};

}

#endif // !ACRO_SHAPE_INSTANCE_MANAGER_H
