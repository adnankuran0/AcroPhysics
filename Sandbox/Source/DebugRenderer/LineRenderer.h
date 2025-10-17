#pragma once

#include "glad/gl.h"
#include "Shader.h"
#include "glm/glm.hpp"
#include <vector>
#include "Acro.h"

class LineRenderer
{
public:
	LineRenderer();
	~LineRenderer();

	inline void AddLine(const glm::vec3& start, const glm::vec3& end, const glm::vec3& color) noexcept
	{
		m_Vertices.push_back({ start,color });
		m_Vertices.push_back({ end,color });
	}

	inline void BeginBatch() noexcept { m_Vertices.clear(); }
	void EndBatch(const Shader& lineShader, const glm::mat4& view, const glm::mat4& proj);


private:
	std::vector<Acro::Debug::DebugVertex> m_Vertices;
	unsigned int m_VAO = 0;
	unsigned int m_VBO = 0;

};
