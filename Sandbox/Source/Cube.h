#pragma once

#include "glm/glm.hpp"
#include <Shader.h>
#include <glm/ext/matrix_transform.hpp>

static constexpr float vertices[] = {
    // Pozisyon          Normal           UV
    // Back face (Z-)
    -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 0.0f,
     0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 1.0f,
     0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 0.0f,
     0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f, 1.0f,
    -0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 0.0f,
    -0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f, 1.0f,

    // Front face (Z+)
    -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f, 0.0f,
     0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f, 0.0f,
     0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f, 1.0f,
     0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f, 1.0f,
    -0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f, 1.0f,
    -0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f, 0.0f,

    // Left face (X-)
    -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 0.0f,
    -0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
    -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
    -0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
    -0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
    -0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f, 0.0f,

    // Right face (X+)
     0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 0.0f,
     0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 1.0f,
     0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
     0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f,
     0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,
     0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f, 0.0f,

     // Bottom face (Y-)
     -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 0.0f,
      0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 0.0f,
      0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 1.0f,
      0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f, 1.0f,
     -0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 1.0f,
     -0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f, 0.0f,

     // Top face (Y+)
     -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 0.0f,
      0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 1.0f,
      0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 0.0f,
      0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f, 1.0f,
     -0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 0.0f,
     -0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f, 1.0f
};


class Cube
{
public:
    Cube() { SetupBuffers(); }
    Cube(const glm::vec3& pos) : m_Pos(pos) { SetupBuffers(); }

    Cube(const Cube&) = delete;
    Cube& operator=(const Cube&) = delete;
    Cube(Cube&& other) noexcept
    {
        m_VAO = other.m_VAO;
        m_VBO = other.m_VBO;
        m_Pos = other.m_Pos;
        m_Rotation = other.m_Rotation;
        other.m_VAO = 0;
        other.m_VBO = 0;
    }

    ~Cube()
    {
        glDeleteVertexArrays(1, &m_VAO);
        glDeleteBuffers(1, &m_VBO);
    }

    void SetPosition(const glm::vec3& pos) { m_Pos = pos; }
    void SetRotation(const glm::quat& rot) { m_Rotation = rot; }
    void SetScale(const glm::vec3& scale) { m_Scale = scale; }

    void Draw(const Shader& shader, const Acro::Rigidbody& body, const glm::mat4& viewMat, const glm::mat4& projMat, const glm::vec3& cameraPos)
    {
        SetPosition(body.GetPosition());
        SetRotation(body.GetOrientation());

        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, m_Pos);
        model *= glm::toMat4(m_Rotation);
        model = glm::scale(model, m_Scale);

        shader.use();
        shader.setMat4("model", model);
        shader.setMat4("view", viewMat);
        shader.setMat4("projection", projMat);
        shader.setVec3("objectColor", 1.0f, 1.0f, 1.0f);
        shader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
        shader.setVec3("lightDir", glm::vec3(-0.2f, -1.0f, -0.3f));
        shader.setVec3("viewPos", cameraPos);
        shader.setInt("tex", 0);

        glBindVertexArray(m_VAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
    }

private:
    void SetupBuffers()
    {
        glGenVertexArrays(1, &m_VAO);
        glGenBuffers(1, &m_VBO);

        glBindVertexArray(m_VAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);
    }

    glm::vec3 m_Pos = glm::vec3(0.0f);
    glm::quat m_Rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    glm::vec3 m_Scale = glm::vec3(1.0f);
    GLuint m_VAO = 0;
    GLuint m_VBO = 0;
};