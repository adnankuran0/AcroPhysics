#ifndef ACRO_INTEGRATOR_H
#define ACRO_INTEGRATOR_H

#include "Math/Vector3.h"

namespace Acro::Physics {

class BodyManager; // forward declaration

class Integrator
{
public:
	Integrator(const Acro::Math::Vector3& gravity = { 0.0f, -9.8f, 0.0f }) : m_Gravity(gravity) {}

	void Step(Acro::Physics::BodyManager& bodyManager, float deltaTime) noexcept;

	inline void SetGravity(const Acro::Math::Vector3& gravity) noexcept { m_Gravity = gravity; }

private:
	void IntegrateBody(Acro::Physics::BodyManager& bodyManager, size_t index, float deltaTime) noexcept;

	Acro::Math::Vector3 m_Gravity;
};

}

#endif // !ACRO_INTEGRATOR_H

