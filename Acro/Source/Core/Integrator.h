#ifndef ACRO_INTEGRATOR_H
#define ACRO_INTEGRATOR_H

#include "Core/Body/BodyManager.h"

namespace Acro::Core {

class Integrator
{
public:
	Integrator(const Acro::Math::Vector3& gravity = { 0.0f, -9.8f, 0.0f }) : m_Gravity(gravity) {}

	void Step(BodyManager& bodyManager, float deltaTime);

	void SetGravity(const Acro::Math::Vector3& gravity) { m_Gravity = gravity; }
	Acro::Math::Vector3 GetGravity() const { return m_Gravity; }

private:
	void IntegrateBody(BodyManager& bodyManager, size_t index, float deltaTime);

	Acro::Math::Vector3 m_Gravity;
};

}

#endif // !ACRO_INTEGRATOR_H

