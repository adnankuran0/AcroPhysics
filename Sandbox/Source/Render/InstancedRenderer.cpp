#include "InstancedRenderer.h"
#include "glad/gl.h"

void InstancedRenderer::Init(const Mesh& mesh, unsigned int maxInstances = 100)
{
	m_Mesh = &mesh;
	m_MaxInstances = maxInstances;

    m_Transforms.reserve(maxInstances);

	glBindVertexArray(m_Mesh->VAO);

	glGenBuffers(1, &m_InstanceVBO);
	glBindBuffer(GL_ARRAY_BUFFER, m_InstanceVBO);
	glBufferData(
		GL_ARRAY_BUFFER,
		maxInstances * sizeof(glm::mat4),
		nullptr,
		GL_DYNAMIC_DRAW
	);

    constexpr GLuint location = 3;
    constexpr std::size_t vec4Size = sizeof(glm::vec4);
    constexpr std::size_t mat4Size = sizeof(glm::mat4);

    for (GLuint i = 0; i < 4; i++)
    {
        glEnableVertexAttribArray(location + i);
        glVertexAttribPointer(
            location + i,
            4,
            GL_FLOAT,
            GL_FALSE,
            mat4Size,
            (void*)(i * vec4Size)
        );
        glVertexAttribDivisor(location + i, 1);
    }

    glBindVertexArray(0);
}

void InstancedRenderer::Begin()
{
    m_Transforms.clear();
}

void InstancedRenderer::Submit(const glm::mat4& transform)
{
    if (m_Transforms.size() >= m_MaxInstances)
        return;

    m_Transforms.push_back(transform);
}



void InstancedRenderer::Draw(const Shader& shader,const glm::mat4& view, const glm::mat4& proj, const glm::vec3& cameraPos) const
{
    if (m_Transforms.size() == 0)
        return;

    glBindBuffer(GL_ARRAY_BUFFER, m_InstanceVBO);
    glBufferSubData(
        GL_ARRAY_BUFFER,
        0,
        m_Transforms.size() * sizeof(glm::mat4),
        m_Transforms.data()
    );

    shader.setMat4("view", view);
    shader.setMat4("projection", proj);
    shader.setInt("tex", 0);

    glBindVertexArray(m_Mesh->VAO);
    glDrawArraysInstanced(
        GL_TRIANGLES,
        0,
        m_Mesh->vertexCount,
        m_Transforms.size()
    );
    glBindVertexArray(0);
}