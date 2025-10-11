#include "Core/World.h"
#include "Core/Rigidbody.h"

using namespace Acro;

void World::Step(float deltaTime)
{
	m_DeltaAccumulator += deltaTime;

	while (m_DeltaAccumulator >= m_FixedDeltaTime && m_StepCount < m_MaxSteps)
	{
		m_Integrator.Step(m_BodyManager.GetData(), m_Gravity, m_FixedDeltaTime);
		m_StepCount++;
		m_DeltaAccumulator -= m_FixedDeltaTime;
	}
	m_StepCount = 0;
	
}

Rigidbody World::CreateBody() noexcept
{
	return Rigidbody(&m_BodyManager, m_BodyManager.CreateBody());
}

void World::DestroyBody(const BodyHandle& handle) noexcept
{
	m_BodyManager.DestroyBody(handle);
}