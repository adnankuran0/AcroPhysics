#ifndef ACRO_BODY_MANAGER_H
#define ACRO_BODY_MANAGER_H

#include <cstdint>
#include "Core/Body/BodyData.h"

namespace Acro::Core {

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
	BodyHandle CreateBody() noexcept ;
	void DestroyBody(const BodyHandle& handle) noexcept;

	inline bool IsValid(const BodyHandle& handle) const noexcept
	{
		if (handle.index >= m_Generations.size()) return false;
		return m_Generations[handle.index] == handle.generation && handle != BodyHandle::Null();
	}

	inline size_t GetBodyCount() const noexcept { return m_BodyData.positions.size(); }

	void AttachShape(const BodyHandle& bodyHandle, const ShapeHandle& shapeHandle);
	void DetachShape(const BodyHandle& bodyHandle);

	inline Acro::Math::Vector3& GetPosition(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.positions[m_Sparse[handle.index]]; }
	inline Acro::Math::Vector3& GetLinearVelocity(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.linearVelocities[m_Sparse[handle.index]]; }
	inline Acro::Math::Quaternion& GetOrientation(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.orientations[m_Sparse[handle.index]]; }
	inline Acro::Math::Vector3& GetAngularVelocity(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.angularVelocities[m_Sparse[handle.index]]; }
	inline Acro::Math::Vector3& GetForceAccumulator(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.forceAccumulators[m_Sparse[handle.index]]; }
	inline float& GetMass(const BodyHandle& handle) noexcept { assert(IsValid(handle)); return m_BodyData.masses[m_Sparse[handle.index]]; }

	inline void SetPosition(const BodyHandle& handle, const Acro::Math::Vector3& pos) noexcept { assert(IsValid(handle)); m_BodyData.positions[m_Sparse[handle.index]] = pos; }
	inline void SetLinearVelocity(const BodyHandle& handle, const Acro::Math::Vector3& linVel) noexcept { assert(IsValid(handle)); m_BodyData.linearVelocities[m_Sparse[handle.index]] = linVel; }
	inline void SetOrientation(const BodyHandle& handle, const Acro::Math::Quaternion& orientation) noexcept { assert(IsValid(handle)); m_BodyData.orientations[m_Sparse[handle.index]] = orientation; }
	inline void SetAngularVelocity(const BodyHandle& handle, const Acro::Math::Vector3& angVel) noexcept { assert(IsValid(handle)); m_BodyData.angularVelocities[m_Sparse[handle.index]] = angVel; }
	inline void ApplyForce(const BodyHandle& handle, const Acro::Math::Vector3& force) noexcept { assert(IsValid(handle)); m_BodyData.forceAccumulators[m_Sparse[handle.index]] += force; }
	inline void SetMass(const BodyHandle& handle, float mass) noexcept { assert(IsValid(handle)); m_BodyData.masses[m_Sparse[handle.index]] = mass; }

	[[nodiscard]] BodyData& GetData() noexcept { return m_BodyData; }

private:
	BodyData m_BodyData; // dense
	std::vector<uint32_t> m_Sparse;
	std::vector<uint32_t> m_DenseToHandle;
	std::vector<uint32_t> m_Generations;
	std::vector<BodyHandle> m_FreeHandles;
};

}

#endif // !ACRO_BODY_MANAGER_H
