#ifndef ACRO_WORLD_H
#define ACRO_WORLD_H

#include "Math/Vector3.h"
#include "Physics/Body/BodyManager.h"
#include "Physics/Shape/ShapeManager.h"
#include "Physics/Shape/ShapeInstanceManager.h"
#include "Physics/Integrator/Integrator.h"
#include "Public/Shape.h"
#include "Debug/DebugRenderer.h"
#include "Public/ShapeInstance.h"
#include "Physics/Pipeline/Broadphase.h"
#include <vector>


namespace Acro {

class World
{
public:
	World(Acro::Math::Vector3 gravity = { 0.0f,-9.8f,0.0f }, unsigned int fixedFPS = 60, unsigned int maxSteps = 8, size_t maxBodies = 1024) : m_Gravity(gravity), 
		m_MaxSteps(maxSteps), m_DeltaAccumulator(0), m_StepCount(0), m_Integrator(gravity)
	{
		m_FixedDeltaTime = 1.0f / float(fixedFPS);
	}

	inline void SetPaused(bool isPaused) { m_IsPaused = isPaused; }

	void Step(float deltaTime);

	inline void SetGravity(const Acro::Math::Vector3& gravity) noexcept { m_Gravity = gravity; }
	inline const Acro::Math::Vector3& GetGravity() const noexcept { return m_Gravity; }

	Rigidbody CreateBody() noexcept; 
	Rigidbody CreateBody(const BodyDescription& desc) noexcept; 
	void DestroyBody(const Rigidbody& body) noexcept;

	Acro::BoxShape CreateBoxShape(const Acro::Math::Vector3& extent = Acro::Math::Vector3(1.0), const Acro::Math::Vector3& offset = Acro::Math::Vector3(0.0)) noexcept;
	Acro::SphereShape CreateSphereShape(float radius = 1.0f, const Acro::Math::Vector3& offset = Acro::Math::Vector3(0.0)) noexcept;

	// TODO: return ShapeInstance wrapper
	Acro::ShapeInstance AttachShape(const Rigidbody& body, const Acro::Shape& shape);
	void DetachShape(const Rigidbody& body, const Acro::Shape& shape);
	void DetachShape(const Rigidbody& body);

	inline [[nodiscard]] Acro::Debug::DebugRenderer& GetDebugRenderer() noexcept { return m_DebugRenderer; }

private:
	void DrawDebugShapes();

	float m_FixedDeltaTime;
	float m_DeltaAccumulator;
	unsigned int m_MaxSteps;
	unsigned int m_StepCount;
	bool m_IsPaused = true;

	Acro::Math::Vector3 m_Gravity;
	Acro::Physics::Integrator m_Integrator;
	Acro::Physics::BodyManager m_BodyManager;
	Acro::Physics::ShapeManager m_ShapeManager;
	Acro::Physics::ShapeInstanceManager m_ShapeInstanceManager;

	std::vector<std::pair<Acro::Physics::ShapeInstanceHandle, Acro::Physics::ShapeInstanceHandle>> m_BroadphaseBuffer;

	Acro::Physics::Broadphase m_Broadphase;

	Acro::Debug::DebugRenderer m_DebugRenderer;
};

}

#endif // !ACRO_WORLD_H
