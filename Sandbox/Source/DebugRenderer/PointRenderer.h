#pragma once
#include "glad/gl.h"
#include "Render/Shader.h"
#include "glm/glm.hpp"
#include <vector>
#include "Acro.h"

class PointRenderer
{
public:
	PointRenderer() = default;
	void Init();
	~PointRenderer();

	inline void BeginBatch() noexcept { m_Vertices.clear(); }
	inline void AddPoint(const glm::vec3& pos, const glm::vec3& color) noexcept { m_Vertices.push_back({ pos,color }); }
	void EndBatch(const Shader& pointShader, const glm::mat4& view, const glm::mat4 proj) noexcept;
	 
private:
	std::vector<Acro::Debug::DebugVertex> m_Vertices;

	unsigned int m_VAO = 0;
	unsigned int m_VBO = 0;

};
