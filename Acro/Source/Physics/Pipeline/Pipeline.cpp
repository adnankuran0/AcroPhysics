#include "Pipeline.h"
#include "Debug/Profiler.h"

using namespace Acro::Debug;

void Pipeline::Execute(float fixedDeltaTime)
{
	Profiler::BeginFrame();

	{
		ProfilerScope integratorScope("Integrator");
		m_Integrator.SetGravity(m_Gravity);
		m_Integrator.Step(m_BodyManager, fixedDeltaTime);
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



	Profiler::EndFrame();
}
