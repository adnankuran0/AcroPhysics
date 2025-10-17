#include "Core/World.h"
#include "Core/Rigidbody.h"

using namespace Acro::Core;
using namespace Acro::Math;

using namespace Acro;

void World::Step(float deltaTime)
{
	if (!m_IsPaused)
	{
		m_DeltaAccumulator += deltaTime;

		while (m_DeltaAccumulator >= m_FixedDeltaTime && m_StepCount < m_MaxSteps)
		{
			m_Integrator.SetGravity(m_Gravity);
			m_Integrator.Step(m_BodyManager.GetData(), m_FixedDeltaTime);

			
			m_ShapeInstanceManager.UpdateWorldData(m_BodyManager, m_ShapeManager);


			m_StepCount++;
			m_DeltaAccumulator -= m_FixedDeltaTime;
		}
		m_StepCount = 0;
	}
	else
	{
		m_ShapeInstanceManager.UpdateWorldData(m_BodyManager, m_ShapeManager);
	}
	
	auto& shapeInstanceData = m_ShapeInstanceManager.GetData();

	if (m_DebugRenderer.drawAABBs)
	{
		for (auto& aabb : shapeInstanceData.worldAABBs)
		{
			Vector3 color = Vector3(1.0f, 0.0f, 0.0f);
			m_DebugRenderer.DrawAABB(aabb.min, aabb.max, color, 1.0f);
		}
	}
	if (m_DebugRenderer.drawShapes)
	{
		for (auto& vertex : shapeInstanceData.worldVertexBuffer)
		{
			Vector3 color = Vector3(0.0f, 1.0f, 0.0f);
			m_DebugRenderer.DrawPoint(vertex, color, 1.0f);
		}

		for (size_t i = 0; i < shapeInstanceData.shapes.size(); i++)
		{
			if (m_ShapeManager.GetShapeType(shapeInstanceData.shapes[i]) == ShapeType::Sphere)
			{
				float radius = m_ShapeManager.GetRadius(shapeInstanceData.shapes[i]);
				Vector3 worldPosition = shapeInstanceData.worldTransforms[i] * Vector3(0.0f);
				m_DebugRenderer.DrawSphere(worldPosition, radius, Vector3(0.0f, 1.0f, 0.0f), 1.0f, 32);
			}
		}
	}
	
	
}

Rigidbody World::CreateBody() noexcept
{
	return Rigidbody(this, &m_BodyManager,&m_ShapeManager, m_BodyManager.CreateBody());
}

Rigidbody World::CreateBody(const BodyDescription& desc) noexcept
{
	return Rigidbody(this, &m_BodyManager, &m_ShapeManager, m_BodyManager.CreateBody(desc));
}

void World::DestroyBody(const Rigidbody& body) noexcept
{
	DetachShape(body);
	m_BodyManager.DestroyBody(body.m_BodyHandle);
}


BoxShape World::CreateBoxShape(const Vector3& extent, const Acro::Math::Vector3& offset) noexcept
{
	return BoxShape(&m_ShapeManager,m_ShapeManager.CreateBoxShape(extent,offset));
}

SphereShape World::CreateSphereShape(float radius, const Acro::Math::Vector3& offset) noexcept
{
	return SphereShape(&m_ShapeManager, m_ShapeManager.CreateSphereShape(radius,offset));
}

ShapeInstanceHandle Acro::World::AttachShape(const Rigidbody& body , const Acro::Shape& shape)
{
	// Check if body is not valid or body already has the shape
	if (!m_ShapeManager.IsValid(shape.m_Handle) 
		|| m_BodyManager.HasShape(body.m_BodyHandle,shape.m_Handle)) 
		return ShapeInstanceHandle::Null();

	m_BodyManager.AttachShape(body.m_BodyHandle, body.m_ShapeHandle);
	m_ShapeManager.AddRef(body.m_ShapeHandle);

	return m_ShapeInstanceManager.CreateShapeInstance(m_ShapeManager,body.m_ShapeHandle, body.m_BodyHandle);
}

void Acro::World::DetachShape(const Rigidbody& body, const Acro::Shape& shape)
{
	if (!m_BodyManager.IsValid(body.m_BodyHandle) || !m_BodyManager.HasShape(body.m_BodyHandle,shape.m_Handle)) return;

	m_BodyManager.DetachShape(body.m_BodyHandle,shape.m_Handle);
	m_ShapeInstanceManager.DestroyShapeInstance(body.m_ShapeInstanceHandle);
	m_ShapeManager.ReleaseRef(body.m_ShapeHandle);

}

void Acro::World::DetachShape(const Rigidbody& body)
{
	if (!m_BodyManager.IsValid(body.m_BodyHandle) || !m_BodyManager.HasShape(body.m_BodyHandle)) return;

	m_BodyManager.DetachShape(body.m_BodyHandle);
	m_ShapeInstanceManager.DestroyShapeInstance(body.m_ShapeInstanceHandle);
	m_ShapeManager.ReleaseRef(body.m_ShapeHandle);
}
