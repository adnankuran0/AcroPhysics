#include "DebugRenderer/PointRenderer.h"

PointRenderer::PointRenderer()
{
	glGenVertexArrays(1, &m_VAO);
	glGenBuffers(1, &m_VBO);
	glBindVertexArray(m_VAO);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)offsetof(Acro::Debug::DebugVertex, color));
	glEnableVertexAttribArray(1);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glEnable(GL_PROGRAM_POINT_SIZE);

}

PointRenderer::~PointRenderer()
{
	glDeleteBuffers(1, &m_VBO);
	glDeleteVertexArrays(1, &m_VAO);
}

void PointRenderer::EndBatch(const Shader& pointShader, const glm::mat4& view, const glm::mat4 proj) noexcept
{
	pointShader.use();
	pointShader.setMat4("view", view);
	pointShader.setMat4("projection", proj);
	glBindVertexArray(m_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
	glBufferData(GL_ARRAY_BUFFER, m_Vertices.size() * sizeof(Acro::Debug::DebugVertex), m_Vertices.data(), GL_STREAM_DRAW);
	glDrawArrays(GL_POINTS, 0, (GLsizei)m_Vertices.size());
}

