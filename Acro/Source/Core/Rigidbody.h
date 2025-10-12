#ifndef ACRO_RIGIDBODY_H
#define ACRO_RIGIDBODY_H

#include "Core/Shape/Shape.h"
#include "Core/Shape/ShapeManager.h"
#include <iostream>

namespace Acro {

class Rigidbody
{
public:
	Rigidbody(Acro::Core::BodyManager* bodyManager, Acro::Core::ShapeManager* shapeManager, Acro::Core::BodyHandle handle) { m_BodyManager = bodyManager; m_ShapeManager = shapeManager;  m_BodyHandle = handle; }
	// TODO: Destructor

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

	void AttachShape(const Acro::Shape& shape) 
	{ 
		if (!m_ShapeManager->IsValid(shape.m_Handle)) return;

		m_ShapeHandle = shape.m_Handle;
		m_BodyManager->AttachShape(m_BodyHandle, m_ShapeHandle);
		m_ShapeManager->AddRef(m_ShapeHandle);
	}

	inline void Destroy() const noexcept 
	{
		m_BodyManager->DestroyBody(m_BodyHandle); 
		m_ShapeManager->ReleaseRef(m_ShapeHandle);
	}
private:

	Acro::Core::BodyHandle m_BodyHandle;
	Acro::Core::ShapeHandle m_ShapeHandle;
	Acro::Core::BodyManager* m_BodyManager;
	Acro::Core::ShapeManager* m_ShapeManager;
};

}

#endif // !ACRO_RIGIDBODY_H
