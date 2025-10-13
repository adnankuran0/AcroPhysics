#pragma once
#include "glad/gl.h"
#include "Shader.h"
#include "glm/glm.hpp"
#include <vector>

class PointRenderer
{
public:
	PointRenderer();
	~PointRenderer();

	void Render(const Shader& shader, const glm::vec3& pos, const glm::vec3& color,
		const glm::mat4 viewMat, const glm::mat4 projMat);

private:
	unsigned int m_VAO = 0;
	unsigned int m_VBO = 0;

};
