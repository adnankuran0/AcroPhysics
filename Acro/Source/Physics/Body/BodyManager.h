#ifndef ACRO_BODY_MANAGER_H
#define ACRO_BODY_MANAGER_H

#include <iostream>
#include <cstdint>

#include "Public/BodyDescription.h"
#include "Physics/Body/BodyData.h"

namespace Acro::Physics {

struct BodyHandle
{
	uint32_t index;
	uint32_t generation;

	static constexpr BodyHandle Null() { return{ std::numeric_limits<uint32_t>::max(),std::numeric_limits<uint32_t>::max() }; }

	bool operator==(const BodyHandle& other) const
	{
		return index == other.index && generation == other.generation;
	}

};

class BodyManager
{
public:
	BodyManager() = default;
	BodyManager(size_t capacity)
	{
		m_BodyData.Reserve(capacity);
	}

	BodyHandle CreateBody() noexcept ;
	BodyHandle CreateBody(const BodyDescription& desc) noexcept;
	void DestroyBody(const BodyHandle& handle) noexcept;

	inline bool IsValid(const BodyHandle& handle) const noexcept
	{
		if (handle.index >= m_Generations.size()) return false;
		return m_Generations[handle.index] == handle.generation && handle != BodyHandle::Null();
	
	}

	inline bool IsDirty(const BodyHandle& handle) const noexcept { return m_BodyData.dirtyFlags[m_Sparse[handle.index]]; }
	inline void SetDirty(const BodyHandle& handle, uint8_t isDirty) noexcept { m_BodyData.dirtyFlags[m_Sparse[handle.index]] = isDirty; }

	inline size_t GetBodyCount() const noexcept { return m_BodyData.positions.size(); }

	void AttachShape(const BodyHandle& bodyHandle, const Acro::Physics::ShapeHandle& shapeHandle);
	void DetachShape(const BodyHandle& bodyHandle, const Acro::Physics::ShapeHandle& shapeHandle);
	void DetachShape(const BodyHandle& bodyHandle);
	bool HasShape(const BodyHandle& bodyHandle, const Acro::Physics::ShapeHandle& shapeHandle);
	bool HasShape(const BodyHandle& bodyHandle);

	inline Acro::Math::Vector3& GetPosition(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.positions[m_Sparse[handle.index]]; }
	inline void SetPosition(const BodyHandle& handle, const Acro::Math::Vector3& pos) noexcept
	{
		assert(IsValid(handle));
		SetDirty(handle, 1);
		m_BodyData.positions[m_Sparse[handle.index]] = pos;
	}

	inline Acro::Math::Vector3& GetLinearVelocity(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.linearVelocities[m_Sparse[handle.index]]; }
	inline void SetLinearVelocity(const BodyHandle& handle, const Acro::Math::Vector3& linVel) noexcept
	{
		assert(IsValid(handle));
		SetDirty(handle, 1);
		m_BodyData.linearVelocities[m_Sparse[handle.index]] = linVel;
	}

	inline Acro::Math::Quaternion& GetOrientation(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.orientations[m_Sparse[handle.index]]; }
	inline void SetOrientation(const BodyHandle& handle, const Acro::Math::Quaternion& orientation) noexcept
	{
		assert(IsValid(handle));
		SetDirty(handle, 1);
		m_BodyData.orientations[m_Sparse[handle.index]] = orientation;
	}

	inline Acro::Math::Vector3& GetAngularVelocity(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.angularVelocities[m_Sparse[handle.index]]; }
	inline void SetAngularVelocity(const BodyHandle& handle, const Acro::Math::Vector3& angVel) noexcept
	{
		assert(IsValid(handle));
		SetDirty(handle, 1);
		m_BodyData.angularVelocities[m_Sparse[handle.index]] = angVel;
	}

	inline Acro::Math::Vector3& GetForceAccumulator(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.forceAccumulators[m_Sparse[handle.index]]; }
	inline void ApplyForce(const BodyHandle& handle, const Acro::Math::Vector3& force) noexcept
	{
		assert(IsValid(handle));
		SetDirty(handle, 1);
		m_BodyData.forceAccumulators[m_Sparse[handle.index]] += force;
	}

	inline float GetMass(const BodyHandle& handle) noexcept 
	{
		assert(IsValid(handle)); 
		float invMass = m_BodyData.inverseMasses[m_Sparse[handle.index]];
		if (invMass <= 0.0f) return 0.0f;
		return 1.0f / invMass;
	}
	inline void SetMass(const BodyHandle& handle, float mass) noexcept
	{
		assert(IsValid(handle));
		float invMass;
		if (mass <= 0.0f)
		{
			invMass = 0.0f;
		}
		else
		{
			invMass = 1.0f / mass;
		}
		m_BodyData.inverseMasses[m_Sparse[handle.index]] = invMass;
	}
	

	[[nodiscard]] BodyData& GetData() noexcept { return m_BodyData; }

private:
	void PushData(const Acro::BodyDescription& desc) noexcept;
	void SwapDenseData(size_t from, size_t to) noexcept;
	void PopBackDenseData() noexcept;
	void CreateHandle(BodyHandle& outHandle, uint32_t& outDenseIndex) noexcept;

	BodyData m_BodyData; // dense
	std::vector<uint32_t> m_Sparse;
	std::vector<uint32_t> m_DenseToHandle;
	std::vector<uint32_t> m_Generations;
	std::vector<BodyHandle> m_FreeHandles;
};

}

#endif // !ACRO_BODY_MANAGER_H
