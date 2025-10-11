#ifndef ACRO_WORLD_H
#define ACRO_WORLD_H

#include "Math/Vector3.h"
#include "Core/BodyManager.h"
#include "Core/Integrator.h"
#include <vector>

using namespace Acro::Math;

namespace Acro {

class Rigidbody; // forward declaration

class World
{
public:
	World(Vector3 gravity = { 0.0f,-9.8f,0.0f }, unsigned int fixedFPS = 60, unsigned int maxSteps = 8) : m_Gravity(gravity), 
		m_MaxSteps(maxSteps), m_DeltaAccumulator(0), m_StepCount(0)
	{
		m_FixedDeltaTime = 1.0f / float(fixedFPS);
	}

	void Step(float deltaTime);

	inline void SetGravity(const Vector3& gravity) noexcept { m_Gravity = gravity; }
	inline Vector3 GetGravity() const noexcept { return m_Gravity; }

	Rigidbody CreateBody() noexcept; 
	void DestroyBody(const BodyHandle& handle) noexcept; 

private:
	float m_FixedDeltaTime;
	float m_DeltaAccumulator;
	unsigned int m_MaxSteps;
	unsigned int m_StepCount;

	Vector3 m_Gravity;
	Integrator m_Integrator;
	BodyManager m_BodyManager;
};

}

#endif // !ACRO_WORLD_H
