#ifndef ACRO_INTEGRATOR_H
#define ACRO_INTEGRATOR_H

#include "Core/BodyManager.h"

namespace Acro::Core {

class Integrator
{
public:
	Integrator(const Vector3& gravity = { 0.0f, -9.8f, 0.0f }) : m_Gravity(gravity) {}

	void Step(BodyData& bodyData, float deltaTime);

	void SetGravity(const Vector3& gravity) { m_Gravity = gravity; }
	Vector3 GetGravity() const { return m_Gravity; }

private:
	void IntegrateBody(BodyData& bodyData, size_t index, float deltaTime);

	Vector3 m_Gravity;
};

}

#endif // !ACRO_INTEGRATOR_H

