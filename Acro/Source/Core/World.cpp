#include "Core/World.h"
#include "Core/Rigidbody.h"

using namespace Acro::Core;
using namespace Acro::Math;

using namespace Acro;

void World::Step(float deltaTime)
{
	m_DeltaAccumulator += deltaTime;

	while (m_DeltaAccumulator >= m_FixedDeltaTime && m_StepCount < m_MaxSteps)
	{
		m_Integrator.SetGravity(m_Gravity);
		m_Integrator.Step(m_BodyManager.GetData(),m_FixedDeltaTime);
		m_StepCount++;
		m_DeltaAccumulator -= m_FixedDeltaTime;
	}
	m_StepCount = 0;
	
}

Rigidbody World::CreateBody() noexcept
{
	return Rigidbody(&m_BodyManager,&m_ShapeManager, m_BodyManager.CreateBody());
}

void World::DestroyBody(const BodyHandle& handle) noexcept
{
	m_BodyManager.DestroyBody(handle);
}

BoxShape World::CreateBoxShape(const Vector3& extent) noexcept
{
	return BoxShape(&m_ShapeManager,m_ShapeManager.CreateBoxShape(extent));
}

SphereShape World::CreateSphereShape(float radius) noexcept
{
	return SphereShape(&m_ShapeManager, m_ShapeManager.CreateSphereShape(radius));
}
