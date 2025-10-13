#include "DebugRenderer/DebugRendererGL.h"

void DebugRendererGL::Init(const char* lineVert, const char* lineFrag,
	const char* pointVert, const char* pointFrag)
{
	m_LineShader = std::make_unique<Shader>(lineVert, lineFrag);
	m_PointShader = std::make_unique<Shader>(pointVert, pointFrag);
}

void DebugRendererGL::Render(Acro::Debug::DebugRenderer& debugRenderer, const glm::mat4 viewMat, const glm::mat4 projMat)
{
	for (const auto& prim : debugRenderer.GetPrimitives())
	{
		switch (prim.type)
		{
		case Acro::Debug::DebugPrimitiveType::Line:
			m_LineRenderer.Render(*m_LineShader, prim.vertices[0].position, prim.vertices[1].position, prim.vertices[0].color, viewMat, projMat);
			break;
		case Acro::Debug::DebugPrimitiveType::Point:
			m_PointRenderer.Render(*m_PointShader, prim.vertices[0].position, prim.vertices[0].color, viewMat, projMat);
			break;
		default:
			break;
		}

	}
}