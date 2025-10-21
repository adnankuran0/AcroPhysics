#ifndef ACRO_PIPELINE_H 
#define ACRO_PIPELINE_H

#include "Math/Vector3.h"
#include <vector>
#include "Physics/Integrator/Integrator.h"
#include <Physics/Shape/ShapeInstanceManager.h>
#include <Physics/Collision/ContactManifold.h>
#include "Narrowphase.h"
#include "Broadphase.h"


class Pipeline
{
public:
	void Execute(float fixedDeltaTime);
private:
	Acro::Math::Vector3 m_Gravity;
	Acro::Physics::Integrator m_Integrator;
	Acro::Physics::BodyManager m_BodyManager;
	Acro::Physics::ShapeManager m_ShapeManager;
	Acro::Physics::ShapeInstanceManager m_ShapeInstanceManager;

	std::vector<std::pair<Acro::Physics::ShapeInstanceHandle, Acro::Physics::ShapeInstanceHandle>> m_BroadphaseBuffer;
	std::vector<Acro::Physics::ContactManifold> m_ContactManifoldBuffer;

	Acro::Physics::Broadphase m_Broadphase;
	Acro::Physics::Narrowphase m_Narrowphase;
};

#endif // !ACRO_PIPELINE_H 

