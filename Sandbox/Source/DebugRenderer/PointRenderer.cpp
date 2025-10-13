#include "DebugRenderer/PointRenderer.h"

PointRenderer::PointRenderer()
{
	glGenVertexArrays(1, &m_VAO);
	glGenBuffers(1, &m_VBO);
	glBindVertexArray(m_VAO);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glEnable(GL_PROGRAM_POINT_SIZE);

}

PointRenderer::~PointRenderer()
{
	glDeleteBuffers(1, &m_VBO);
	glDeleteVertexArrays(1, &m_VAO);
}

void PointRenderer::Render(const Shader& shader, const glm::vec3& pos, const glm::vec3& color,
	const glm::mat4 viewMat, const glm::mat4 projMat)
{
	shader.use();

	glm::mat4 modelMat = glm::mat4(1.0f);

	shader.setVec3("color", color);

	shader.setMat4("model", modelMat);
	shader.setMat4("view", viewMat);
	shader.setMat4("projection", projMat);

	std::vector<float> vertex = { pos.x,pos.y,pos.z };
	glBindVertexArray(m_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
	glBufferData(GL_ARRAY_BUFFER, vertex.size() * sizeof(float), vertex.data(), GL_STATIC_DRAW);
	glDrawArrays(GL_POINTS, 0, 1);

}