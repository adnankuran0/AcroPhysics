#ifndef ACRO_RIGIDBODY_H
#define ACRO_RIGIDBODY_H

#include "Public/Shape.h"
#include "Physics/Shape/ShapeManager.h"
#include "Public/ShapeInstance.h"
#include <iostream>
#include "CollisionLayer.h"

namespace Acro {

class World;

class Rigidbody
{
public:
	Rigidbody(Acro::World* world, Acro::Physics::BodyManager* bodyManager, Acro::Physics::ShapeManager* shapeManager, Acro::Physics::BodyHandle handle) :
		m_ShapeHandle(Acro::Physics::ShapeHandle::Null()),
		m_BodyHandle(Acro::Physics::BodyHandle::Null()),
		m_ShapeInstanceHandle(Acro::Physics::ShapeInstanceHandle::Null())
	{ 
		m_World = world; 
		m_BodyManager = bodyManager; 
		m_ShapeManager = shapeManager;  
		m_BodyHandle = handle; 
	}

	Rigidbody() = default;

	inline bool IsValid() const noexcept { return m_BodyManager && m_ShapeManager && m_BodyManager->IsValid(m_BodyHandle); }

	inline const Acro::Math::Vector3& GetPosition()		            const noexcept { return m_BodyManager->GetPosition(m_BodyHandle); }
	inline const Acro::Math::Vector3& GetLinearVelocity()           const noexcept { return m_BodyManager->GetLinearVelocity(m_BodyHandle); }
	inline const Acro::Math::Quaternion& GetOrientation()           const noexcept { return m_BodyManager->GetOrientation(m_BodyHandle); }
	inline const Acro::Math::Vector3& GetAngularVelocity()          const noexcept { return m_BodyManager->GetAngularVelocity(m_BodyHandle); }
	inline const Acro::Math::Vector3& GetForceAccumulator()         const noexcept { return m_BodyManager->GetForceAccumulator(m_BodyHandle); }
	inline const float GetMass()				        const noexcept { return m_BodyManager->GetMass(m_BodyHandle); }

	inline void SetPosition(const Acro::Math::Vector3& pos)				  noexcept { m_BodyManager->SetPosition(m_BodyHandle,pos); }
	inline void SetLinearVelocity(const Acro::Math::Vector3& linVel)	  noexcept { m_BodyManager->SetLinearVelocity(m_BodyHandle, linVel); }
	inline void SetOrientation(const Acro::Math::Quaternion& orientation) noexcept { m_BodyManager->SetOrientation(m_BodyHandle,orientation); }
	inline void SetAngularVelocity(const Acro::Math::Vector3& angVel)	  noexcept { m_BodyManager->SetAngularVelocity(m_BodyHandle,angVel); }
	inline void ApplyForce(const Acro::Math::Vector3& force)			  noexcept { m_BodyManager->ApplyForce(m_BodyHandle,force); }
	inline void SetMass(float mass)							  noexcept { m_BodyManager->SetMass(m_BodyHandle,mass); }

	inline Acro::ShapeType GetShapeType() const noexcept
	{
		if (!m_ShapeManager || !m_ShapeManager->IsValid(m_ShapeHandle))
			return Acro::ShapeType::Null;

		return m_ShapeManager->GetShapeType(m_ShapeHandle);
	}

	Acro::ShapeInstance AttachShape(const Acro::Shape& shape);

	void DetachShape(const Acro::Shape& shape);

	
private:
	Acro::Physics::BodyHandle m_BodyHandle;
	Acro::Physics::ShapeHandle m_ShapeHandle;
	Acro::Physics::ShapeInstanceHandle m_ShapeInstanceHandle;

	Acro::Physics::BodyManager* m_BodyManager;
	Acro::Physics::ShapeManager* m_ShapeManager;

	Acro::World* m_World;

	friend class World;

};

}

#endif // !ACRO_RIGIDBODY_H
