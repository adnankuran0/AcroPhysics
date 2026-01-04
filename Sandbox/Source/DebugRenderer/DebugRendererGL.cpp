#include "DebugRenderer/DebugRendererGL.h"

void DebugRendererGL::Init(const char* lineVert, const char* lineFrag,
	const char* pointVert, const char* pointFrag)
{
	m_LineShader = std::make_unique<Shader>(lineVert, lineFrag);
	m_PointShader = std::make_unique<Shader>(pointVert, pointFrag);
	m_LineRenderer.Init();
	m_PointRenderer.Init();
}

void DebugRendererGL::Render(Acro::Debug::DebugRenderer& debugRenderer, const glm::mat4 viewMat, const glm::mat4 projMat,float dt)
{
	m_LineRenderer.BeginBatch();
	m_PointRenderer.BeginBatch();

	for (const auto& prim : debugRenderer.GetPrimitives())
	{
		switch (prim.type)
		{
		case Acro::Debug::DebugPrimitiveType::Line:
			m_LineRenderer.AddLine(prim.vertices[0].position, prim.vertices[1].position, prim.vertices[0].color);
			break;
		case Acro::Debug::DebugPrimitiveType::Point:
			m_PointRenderer.AddPoint(prim.vertices[0].position, prim.vertices[0].color);
			break;
		default:
			break;
		}
	}

	m_LineRenderer.EndBatch(*m_LineShader,viewMat,projMat);
	m_PointRenderer.EndBatch(*m_PointShader, viewMat, projMat);

	debugRenderer.Clear(dt);
}