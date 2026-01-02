#include "Public/World.h"
#include "Public/Rigidbody.h"
#include "Debug/Profiler.h"

using namespace Acro::Math;
using namespace Acro::Physics;
using namespace Acro::Debug;
using namespace Acro;

void World::Step(float deltaTime) noexcept
{
	if (!m_IsPaused)
	{
		m_DeltaAccumulator += deltaTime;

		while (m_DeltaAccumulator >= m_FixedDeltaTime && m_StepCount < m_MaxSteps)
		{
			Profiler::BeginFrame();

			{
				ProfilerScope integratorScope("Integrator");
				m_Integrator.SetGravity(m_Gravity);
				m_Integrator.Step(m_BodyManager, m_FixedDeltaTime);
			}
			
			{
				ProfilerScope shapeUpdatesScope("Shape updates");
				m_ShapeInstanceManager.UpdateWorldData(m_BodyManager, m_ShapeManager);
			}

			{
				ProfilerScope broadphaseScope("Broadphase");
				m_BroadphaseBuffer = m_Broadphase.Compute(&m_ShapeInstanceManager);
			}

			{
				ProfilerScope narrowphaseScope("Narrowphase");
				m_ContactManifoldBuffer = m_Narrowphase.Compute(m_ShapeManager, m_ShapeInstanceManager, m_BroadphaseBuffer);
			}

			{
				ProfilerScope solverScope("Solver");
				m_Solver.Solve(m_ContactManifoldBuffer, m_ShapeInstanceManager, m_BodyManager,m_FixedDeltaTime);
			}

			Profiler::EndFrame();
			Profiler::PrintSummary();

			m_StepCount++;
			m_DeltaAccumulator -= m_FixedDeltaTime;
		}
		m_StepCount = 0;
	}
	else
	{
		m_ShapeInstanceManager.UpdateWorldData(m_BodyManager, m_ShapeManager);
	}
	
	DrawDebugShapes();
	
	

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

ShapeInstance Acro::World::AttachShape(const Rigidbody& body , const Acro::Shape& shape)
{
	// Check if body is not valid or body already has the shape
	assert(m_ShapeManager.IsValid(shape.m_Handle));
	assert(!m_BodyManager.HasShape(body.m_BodyHandle, shape.m_Handle));

	m_BodyManager.AttachShape(body.m_BodyHandle,shape.m_Handle);
	m_ShapeManager.AddRef(body.m_ShapeHandle);
	ShapeInstanceHandle shapeInstanceHandle = m_ShapeInstanceManager.CreateShapeInstance(m_ShapeManager, body.m_ShapeHandle, body.m_BodyHandle);

	// enable layer 1 by default
	m_ShapeInstanceManager.EnableLayer(shapeInstanceHandle, 1);

	return ShapeInstance(&m_ShapeInstanceManager, shapeInstanceHandle);
}

void Acro::World::DetachShape(const Rigidbody& body, const Acro::Shape& shape)
{
	if (!m_BodyManager.IsValid(body.m_BodyHandle) || !m_BodyManager.HasShape(body.m_BodyHandle,shape.m_Handle)) return;

	m_BodyManager.DetachShape(body.m_BodyHandle,shape.m_Handle);
	m_ShapeInstanceManager.DestroyShapeInstance(body.m_ShapeInstanceHandle);
	
	int shapeRefCount = m_ShapeManager.ReleaseRef(body.m_ShapeHandle);
	if (shapeRefCount <= 0)
		m_ShapeManager.DestroyShape(shape.m_Handle);

}

void Acro::World::DetachShape(const Rigidbody& body)
{
	if (!m_BodyManager.IsValid(body.m_BodyHandle) || !m_BodyManager.HasShape(body.m_BodyHandle)) return;

	m_BodyManager.DetachShape(body.m_BodyHandle);
	m_ShapeInstanceManager.DestroyShapeInstance(body.m_ShapeInstanceHandle);
	m_ShapeManager.ReleaseRef(body.m_ShapeHandle);
}

void Acro::World::DrawDebugShapes()
{
	auto& shapeInstanceData = m_ShapeInstanceManager.GetData();

	if (m_DebugRenderer.drawContactPoints)
	{
		Vector3 color = Vector3(1.0f, 1.0f, 0.0f);
		for (auto& cp : m_ContactManifoldBuffer)
		{
			for (int i = 0; i < cp.count; i++)
			{
				const auto& pos = cp.points[i].position;
				const auto& nor = cp.points[i].normal;

				m_DebugRenderer.DrawPoint(pos, color, 0.5f);
				m_DebugRenderer.DrawLine(pos, pos + nor * 0.5f, color, 0.5f);
			}
		}
	}

	if (m_DebugRenderer.drawAABBs)
	{
		for (auto& aabb : shapeInstanceData.worldAABBs)
		{
			Vector3 color = Vector3(0.0f, 0.0f, 1.0f);
			m_DebugRenderer.DrawAABB(aabb.min, aabb.max, color);
		}
		for (auto& pair : m_BroadphaseBuffer)
		{
			auto firstAABB = m_ShapeInstanceManager.GetWorldAABB(pair.first);
			auto secondAABB = m_ShapeInstanceManager.GetWorldAABB(pair.second);
			Vector3 color(1.0f, 0.0f, 0.0f);
			// TODO: it doesnt work with frame time duration
			m_DebugRenderer.DrawAABB(firstAABB.min, firstAABB.max, color, 0.05f);
			m_DebugRenderer.DrawAABB(secondAABB.min, secondAABB.max, color, 0.05f);
		}
	}
	
	if (m_DebugRenderer.drawShapes)
	{
		for (auto& vertex : shapeInstanceData.worldVertexBuffer)
		{
			Vector3 color = Vector3(0.0f, 1.0f, 0.0f);
			m_DebugRenderer.DrawPoint(vertex, color);
		}

		for (size_t i = 0; i < shapeInstanceData.shapes.size(); i++)
		{
			if (m_ShapeManager.GetShapeType(shapeInstanceData.shapes[i]) == ShapeType::Sphere)
			{
				float radius = m_ShapeManager.GetRadius(shapeInstanceData.shapes[i]);
				Vector3 worldPosition = shapeInstanceData.worldTransforms[i] * Vector3(0.0f);
				m_DebugRenderer.DrawSphere(worldPosition, radius, Vector3(0.0f, 1.0f, 0.0f));

			}
		}
	}
}
