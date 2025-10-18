#ifndef ACRO_RIGIDBODY_H
#define ACRO_RIGIDBODY_H

#include "Core/Shape/Shape.h"
#include "Core/Shape/ShapeManager.h"
#include "Core/Shape/ShapeInstance.h"
#include <iostream>
#include "CollisionLayer.h"

namespace Acro {

class Rigidbody
{
public:
	Rigidbody(Acro::World* world, Acro::Core::BodyManager* bodyManager, Acro::Core::ShapeManager* shapeManager, Acro::Core::BodyHandle handle) :
		m_ShapeHandle(Acro::Core::ShapeHandle::Null()), 
		m_BodyHandle(Acro::Core::BodyHandle::Null()),
		m_ShapeInstanceHandle(Acro::Core::ShapeInstanceHandle::Null())
	{ 
		m_World = world; 
		m_BodyManager = bodyManager; 
		m_ShapeManager = shapeManager;  
		m_BodyHandle = handle; 
	}

	inline bool IsValid() const noexcept { return m_BodyManager && m_ShapeInstanceManager && m_ShapeManager && m_BodyManager->IsValid(m_BodyHandle); }

	inline const Acro::Math::Vector3& GetPosition()		            const noexcept { return m_BodyManager->GetPosition(m_BodyHandle); }
	inline const Acro::Math::Vector3& GetLinearVelocity()           const noexcept { return m_BodyManager->GetLinearVelocity(m_BodyHandle); }
	inline const Acro::Math::Quaternion& GetOrientation()           const noexcept { return m_BodyManager->GetOrientation(m_BodyHandle); }
	inline const Acro::Math::Vector3& GetAngularVelocity()          const noexcept { return m_BodyManager->GetAngularVelocity(m_BodyHandle); }
	inline const Acro::Math::Vector3& GetForceAccumulator()         const noexcept { return m_BodyManager->GetForceAccumulator(m_BodyHandle); }
	inline const float& GetMass()				        const noexcept { return m_BodyManager->GetMass(m_BodyHandle); }

	inline void SetPosition(const Acro::Math::Vector3& pos)				  noexcept { m_BodyManager->SetPosition(m_BodyHandle,pos); }
	inline void SetLinearVelocity(const Acro::Math::Vector3& linVel)	  noexcept { m_BodyManager->SetLinearVelocity(m_BodyHandle, linVel); }
	inline void SetOrientation(const Acro::Math::Quaternion& orientation) noexcept { m_BodyManager->SetOrientation(m_BodyHandle,orientation); }
	inline void SetAngularVelocity(const Acro::Math::Vector3& angVel)	  noexcept { m_BodyManager->SetAngularVelocity(m_BodyHandle,angVel); }
	inline void ApplyForce(const Acro::Math::Vector3& force)			  noexcept { m_BodyManager->ApplyForce(m_BodyHandle,force); }
	inline void SetMass(float mass)							  noexcept { m_BodyManager->SetMass(m_BodyHandle,mass); }

	Acro::ShapeInstance AttachShape(const Acro::Shape& shape) 
	{ 
		m_ShapeHandle = shape.m_Handle;
		return m_World->AttachShape(*this,shape);
	}

	void DetachShape(const Acro::Shape& shape)
	{
		m_World->DetachShape(*this, shape);
	}

	
private:
	Acro::Core::BodyHandle m_BodyHandle;
	Acro::Core::ShapeHandle m_ShapeHandle;
	Acro::Core::ShapeInstanceHandle m_ShapeInstanceHandle;

	Acro::Core::BodyManager* m_BodyManager;
	Acro::Core::ShapeManager* m_ShapeManager;
	Acro::Core::ShapeInstanceManager* m_ShapeInstanceManager;

	Acro::World* m_World;

	friend class World;

};

}

#endif // !ACRO_RIGIDBODY_H
