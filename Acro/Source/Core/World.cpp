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

		m_ShapeInstanceManager.UpdateWorldData(m_BodyManager, m_ShapeManager);

		m_StepCount++;
		m_DeltaAccumulator -= m_FixedDeltaTime;
	}
	m_StepCount = 0;
	
}

Rigidbody World::CreateBody() noexcept
{
	return Rigidbody(this, &m_BodyManager,&m_ShapeManager, m_BodyManager.CreateBody());
}

void World::DestroyBody(const Rigidbody& body) noexcept
{
	DetachShape(body);
	m_BodyManager.DestroyBody(body.m_BodyHandle);

	// Shape may not have a shape instance
	//m_ShapeInstanceManager->DestroyShapeInstance(m_ShapeInstanceHandle);
}


BoxShape World::CreateBoxShape(const Vector3& extent) noexcept
{
	return BoxShape(&m_ShapeManager,m_ShapeManager.CreateBoxShape(extent));
}

SphereShape World::CreateSphereShape(float radius) noexcept
{
	return SphereShape(&m_ShapeManager, m_ShapeManager.CreateSphereShape(radius));
}

ShapeInstanceHandle Acro::World::AttachShape(const Rigidbody& body , const Acro::Shape& shape)
{
	if (!m_ShapeManager.IsValid(shape.m_Handle)) return ShapeInstanceHandle::Null();

	//m_ShapeHandle = shape.m_Handle;
	m_BodyManager.AttachShape(body.m_BodyHandle, body.m_ShapeHandle);
	m_ShapeManager.AddRef(body.m_ShapeHandle);

	return m_ShapeInstanceManager.CreateShapeInstance(body.m_ShapeHandle, body.m_BodyHandle);
}

void Acro::World::DetachShape(const Rigidbody& body)
{
	if (!m_BodyManager.IsValid(body.m_BodyHandle)) return;

	m_BodyManager.DetachShape(body.m_BodyHandle);
	m_ShapeManager.ReleaseRef(body.m_ShapeHandle);
	m_ShapeInstanceManager.DestroyShapeInstance(body.m_ShapeInstanceHandle);

}
