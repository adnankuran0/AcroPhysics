#include "DebugRenderer/LineRenderer.h"

#include <iostream>
#include <Acro.h>

LineRenderer::LineRenderer()
{
	glGenVertexArrays(1, &m_VAO);
	glGenBuffers(1, &m_VBO);
	glBindVertexArray(m_VAO);

	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)offsetof(Acro::Debug::DebugVertex,color));
	glEnableVertexAttribArray(1);
	

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
}

LineRenderer::~LineRenderer()
{
	glDeleteBuffers(1, &m_VBO);
	glDeleteVertexArrays(1, &m_VAO);
}

void LineRenderer::EndBatch(const Shader& lineShader, const glm::mat4& view, const glm::mat4& proj)
{
	lineShader.use();
	lineShader.setMat4("view", view);
	lineShader.setMat4("projection", proj);
	glBindVertexArray(m_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
	glBufferData(GL_ARRAY_BUFFER, m_Vertices.size() * sizeof(Acro::Debug::DebugVertex), m_Vertices.data(), GL_STREAM_DRAW);
	glDrawArrays(GL_LINES, 0,(GLsizei)m_Vertices.size());
}

