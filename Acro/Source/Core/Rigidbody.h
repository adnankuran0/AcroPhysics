#ifndef ACRO_RIGIDBODY_H
#define ACRO_RIGIDBODY_H

#include "Core/World.h"

using namespace Acro;

namespace Acro {

class Rigidbody
{
public:
	Rigidbody() : m_BodyManager(nullptr) {}
	Rigidbody(BodyManager* bodyManagaer, BodyHandle handle) { m_BodyManager = bodyManagaer; m_Handle = handle; }

	inline const Vector3& GetPosition() const noexcept { return m_BodyManager->GetPosition(m_Handle); }
	inline const Vector3& GetLinearVelocity() const noexcept { return m_BodyManager->GetLinearVelocity(m_Handle); }
	inline const Quaternion& GetOrientation() const noexcept { return m_BodyManager->GetOrientation(m_Handle); }
	inline const Vector3& GetAngularVelocity() const noexcept { return m_BodyManager->GetAngularVelocity(m_Handle); }
	inline const Vector3& GetForceAccumulator() const noexcept { return m_BodyManager->GetForceAccumulator(m_Handle); }
	inline const Vector3& GetMass() const noexcept { return m_BodyManager->GetMass(m_Handle); }

	inline void SetPosition(const Vector3& pos) const noexcept { m_BodyManager->SetPosition(m_Handle,pos); }
	inline void SetLinearVelocity(const Vector3& linVel) const noexcept { m_BodyManager->SetLinearVelocity(m_Handle, linVel); }
	inline void SetOrientation(const Quaternion& orientation) const noexcept { m_BodyManager->SetOrientation(m_Handle,orientation); }
	inline void SetAngularVelocity(const Vector3& angVel) const noexcept { m_BodyManager->SetAngularVelocity(m_Handle,angVel); }
	inline void ApplyForce(const Vector3& force) const noexcept { m_BodyManager->ApplyForce(m_Handle,force); }
	inline void SetMass(float mass) const noexcept { m_BodyManager->SetMass(m_Handle,mass); }

	inline void Destroy() const noexcept { m_BodyManager->DestroyBody(m_Handle); }
private:
	BodyHandle m_Handle;
	BodyManager* m_BodyManager;
};

}

#endif // !ACRO_RIGIDBODY_H
