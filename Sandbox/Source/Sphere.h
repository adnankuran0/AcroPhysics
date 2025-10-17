#pragma once

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include <Shader.h>
#include <glm/ext/matrix_transform.hpp>
#include <vector>
#include <Acro.h>

class Sphere
{
public:
    Sphere(int nSlices = 16, int nStacks = 16, float radius = 0.5f) : m_Slices(nSlices), m_Stacks(nStacks), m_Radius(radius) 
    {
        GenerateMesh();
        SetupBuffers();
    }
    Sphere(const glm::vec3& pos,int nSlices = 16, int nStacks = 16, float radius = 0.5f) : m_Pos(pos), m_Slices(nSlices), m_Stacks(nStacks), m_Radius(radius)
    {
        GenerateMesh();
        SetupBuffers();
    }

    Sphere(const Sphere&) = delete;
    Sphere& operator=(const Sphere&) = delete;

    Sphere(Sphere&& other) noexcept
    {
        m_VAO = other.m_VAO;
        m_VBO = other.m_VBO;
        m_Pos = other.m_Pos;
        m_Rotation = other.m_Rotation;
        m_VertexCount = other.m_VertexCount;
        m_Radius = other.m_Radius;
        m_Slices = other.m_Slices;
        m_Stacks = other.m_Stacks;
        other.m_VAO = 0;
        other.m_VBO = 0;
    }

    ~Sphere()
    {
        glDeleteVertexArrays(1,&m_VAO);
        glDeleteBuffers(1, &m_VBO);
    }

    inline void SetPosition(const glm::vec3& pos) noexcept { m_Pos = pos; }
    inline void SetRotation(const glm::quat& rot) noexcept { m_Rotation = rot; }

    void Draw(const Shader& shader, const Acro::Rigidbody& body, const glm::mat4& viewMat, const glm::mat4& projMat, const glm::vec3& cameraPos)
    {
        SetPosition(body.GetPosition());
        SetRotation(body.GetOrientation());

        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, m_Pos);
        model *= glm::toMat4(m_Rotation);

        shader.use();
        shader.setMat4("model", model);
        shader.setMat4("view", viewMat);
        shader.setMat4("projection", projMat);
        shader.setVec3("objectColor", 1.0f, 1.0f, 1.0f);
        shader.setVec3("lightColor", 1.0f, 1.0f, 1.0f);
        shader.setVec3("lightPos", glm::vec3(0.5f, 3.0f, 2.0f));
        shader.setVec3("viewPos", cameraPos);

        glBindVertexArray(m_VAO);
        glDrawArrays(GL_TRIANGLES, 0, m_VertexCount);
    }

private:

    void GenerateMesh() 
    {
        m_Vertices.clear();

        glm::vec3 top(0.0f,m_Radius,0.0f);
        glm::vec3 bottom(0.0f, -m_Radius, 0.0f);

        std::vector<glm::vec3> verts;
        verts.push_back(top);
        for (int stack = 1; stack < m_Stacks; stack++)
        {
            float phi = glm::pi<float>() * stack / m_Stacks;
            for (int slice = 0; slice < m_Slices; slice++)
            {
                float theta = 2.0f * glm::pi<float>() * slice / m_Slices;
                float x = m_Radius * sin(phi) * cos(theta);
                float y = m_Radius * cos(phi);
                float z = m_Radius * sin(phi) * sin(theta);
                verts.push_back(glm::vec3(x,y,z));
            }
;        }
        verts.push_back(bottom);

        // top triangles
        for (int i = 0; i < m_Slices; i++)
        {
            int i0 = 0;
            int i1 = i + 1;
            int i2 = (i + 1) % m_Slices + 1;
            PushTriangle(verts[i2], verts[i1], verts[i0]);
        }

        // bottom triangles
        int bottomIndex = static_cast<int>(verts.size() - 1);
        int lastStackStart = bottomIndex - m_Slices;
        for (int i = 0; i < m_Slices; i++)
        {
            int i0 = bottomIndex;
            int i1 = lastStackStart + (i + 1) % m_Slices;
            int i2 = lastStackStart + i;
            PushTriangle(verts[i2], verts[i1], verts[i0]);
        }

        //middle quads
        for (int stack = 0; stack < m_Stacks - 2; stack++)
        {
            int start0 = 1 + stack * m_Slices;
            int start1 = start0 + m_Slices;
            for (int slice = 0; slice < m_Slices; slice++)
            {
                int i0 = start0 + slice;
                int i1 = start0 + (slice + 1) % m_Slices;
                int i2 = start1+ (slice + 1) % m_Slices;
                int i3 = start1 + slice;

                PushTriangle(verts[i0], verts[i1], verts[i2]);
                PushTriangle(verts[i2], verts[i3], verts[i0]);
            }
        }

        m_VertexCount = static_cast<GLsizei>(m_Vertices.size() / 6);
    }

    void PushTriangle(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2)
    {
        auto pushVertex = [this](const glm::vec3& v)
            {
                glm::vec3 n = glm::normalize(v);
                m_Vertices.push_back(v.x); m_Vertices.push_back(v.y); m_Vertices.push_back(v.z);
                m_Vertices.push_back(n.x); m_Vertices.push_back(n.y); m_Vertices.push_back(n.z);
            };

        pushVertex(v0);
        pushVertex(v1);
        pushVertex(v2);
    }

    void SetupBuffers()
    {
        glGenVertexArrays(1, &m_VAO);
        glGenBuffers(1, &m_VBO);

        glBindVertexArray(m_VAO);
        glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
        glBufferData(GL_ARRAY_BUFFER, m_Vertices.size() * sizeof(float), m_Vertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
    }

    std::vector<float> m_Vertices;
    GLsizei m_VertexCount;

    glm::vec3 m_Pos = glm::vec3(0.0f);
    glm::quat m_Rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    GLuint m_VAO = 0;
    GLuint m_VBO = 0;

    int m_Slices;
    int m_Stacks;
    float m_Radius;
};