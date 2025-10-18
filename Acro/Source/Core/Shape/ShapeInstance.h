#ifndef ACRO_SHAPE_INSTANCE_H
#define ACRO_SHAPE_INSTANCE_H

#include "ShapeInstanceManager.h"

namespace Acro{

class ShapeInstance
{
public:
	ShapeInstance(Acro::Core::ShapeInstanceManager* shapeInstanceManager, const Acro::Core::ShapeInstanceHandle& shapeInstanceHandle) :
		m_ShapeInstanceManager(shapeInstanceManager), m_Handle(shapeInstanceHandle) {}

	inline bool IsValid() const noexcept { return m_ShapeInstanceManager && m_ShapeInstanceManager->IsValid(m_Handle); }

	inline const Acro::Math::Matrix4& GetWorldTransform() const noexcept { return m_ShapeInstanceManager->GetWorldTransform(m_Handle); }
	inline const Acro::Math::AABB& GetWorldAABB() const noexcept { return m_ShapeInstanceManager->GetWorldAABB(m_Handle); }

	inline void SetCollisionFilter(const Acro::CollisionFilter& filter) { m_ShapeInstanceManager->SetCollisionFilter(m_Handle, filter); }
	inline const Acro::CollisionFilter& GetCollisionFilter() noexcept { return m_ShapeInstanceManager->GetCollisionFilter(m_Handle); }

	inline void EnableLayer(uint32_t index) noexcept { m_ShapeInstanceManager->EnableLayer(m_Handle, index); }
	inline void DisableLayer(uint32_t index) noexcept { m_ShapeInstanceManager->DisableLayer(m_Handle, index); }
	inline void EnableMask(uint32_t index) noexcept { m_ShapeInstanceManager->EnableMask(m_Handle, index); }
	inline void DisableMask(uint32_t index) noexcept { m_ShapeInstanceManager->DisableMask(m_Handle, index); }

private:
	Acro::Core::ShapeInstanceManager* m_ShapeInstanceManager;
	Acro::Core::ShapeInstanceHandle m_Handle;

};

}

#endif // !ACRO_SHAPE_INSTANCE_H
