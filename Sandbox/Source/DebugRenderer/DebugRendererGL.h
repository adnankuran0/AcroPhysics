#pragma once

#include "Acro.h"
#include <glad/gl.h>
#include "LineRenderer.h"
#include "PointRenderer.h"
#include <memory>

class DebugRendererGL
{
public:
	void Init(const char* lineVert, const char* lineFrag,
		const char* pointVert, const char* pointFrag);

	void Render(Acro::Debug::DebugRenderer& debugRenderer, const glm::mat4 viewMat, const glm::mat4 projMat);

private:
	std::unique_ptr<Shader> m_LineShader;
	std::unique_ptr<Shader> m_PointShader;
	LineRenderer m_LineRenderer;
	PointRenderer m_PointRenderer;
};
