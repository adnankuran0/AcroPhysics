#include "DebugRenderer/LineRenderer.h"

#include <iostream>

LineRenderer::LineRenderer()
{
	glGenVertexArrays(1, &m_VAO);
	glGenBuffers(1, &m_VBO);
	glBindVertexArray(m_VAO);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

LineRenderer::~LineRenderer()
{
	glDeleteBuffers(1, &m_VBO);
	glDeleteVertexArrays(1, &m_VAO);
}

void LineRenderer::Render(const Shader& shader, const glm::vec3& start, const glm::vec3& end, const glm::vec3& color,
	const glm::mat4 viewMat, const glm::mat4 projMat)
{
	shader.use();

	glm::mat4 modelMat = glm::mat4(1.0f);

	shader.setVec3("color", color);

	shader.setMat4("model", modelMat);
	shader.setMat4("view", viewMat);
	shader.setMat4("projection", projMat);

	std::vector<float> vertices = { start.x,start.y,start.z,end.x,end.y,end.z };
	glBindVertexArray(m_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
	glDrawArrays(GL_LINES, 0, 2);
}
