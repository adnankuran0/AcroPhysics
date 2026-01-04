#pragma once

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"
#include "Render/Shader.h"
#include <glm/ext/matrix_transform.hpp>
#include <vector>
#include <Acro.h>
#include "Mesh.h"

class Sphere : public Mesh
{
public:
    Sphere(int nSlices = 16, int nStacks = 16, float radius = 0.5f) : m_Slices(nSlices), m_Stacks(nStacks), m_Radius(radius)
    {
        GenerateMesh();
        SetupBuffers();
    }
   
    Sphere(const Sphere&) = delete;
    Sphere& operator=(const Sphere&) = delete;

    Sphere(Sphere&& other) noexcept
    {
        VAO = other.VAO;
        VBO = other.VBO;
        vertexCount = other.vertexCount;
        m_Radius = other.m_Radius;
        m_Slices = other.m_Slices;
        m_Stacks = other.m_Stacks;
        other.VAO = 0;
        other.VBO = 0;
    }

    ~Sphere()
    {
        glDeleteVertexArrays(1, &VAO);
        glDeleteBuffers(1, &VBO);
    }

private:

    struct VertexData {
        glm::vec3 pos;
        glm::vec2 uv;
    };

    void GenerateMesh()
    {
        m_Vertices.clear();

        glm::vec3 top(0.0f, m_Radius, 0.0f);
        glm::vec3 bottom(0.0f, -m_Radius, 0.0f);

        std::vector<VertexData> verts;

        verts.push_back({ top, glm::vec2(0.5f, 1.0f) });

        for (int stack = 1; stack < m_Stacks; stack++)
        {
            float phi = glm::pi<float>() * stack / m_Stacks;
            float v = 1.0f - (float)stack / m_Stacks;

            for (int slice = 0; slice < m_Slices; slice++)
            {
                float theta = 2.0f * glm::pi<float>() * slice / m_Slices;
                float u = (float)slice / m_Slices;

                float x = m_Radius * sin(phi) * cos(theta);
                float y = m_Radius * cos(phi);
                float z = m_Radius * sin(phi) * sin(theta);

                verts.push_back({ glm::vec3(x, y, z), glm::vec2(u, v) });
            }
        }

        verts.push_back({ bottom, glm::vec2(0.5f, 0.0f) });

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

        // middle quads
        for (int stack = 0; stack < m_Stacks - 2; stack++)
        {
            int start0 = 1 + stack * m_Slices;
            int start1 = start0 + m_Slices;
            for (int slice = 0; slice < m_Slices; slice++)
            {
                int i0 = start0 + slice;
                int i1 = start0 + (slice + 1) % m_Slices;
                int i2 = start1 + (slice + 1) % m_Slices;
                int i3 = start1 + slice;

                PushTriangle(verts[i0], verts[i1], verts[i2]);
                PushTriangle(verts[i2], verts[i3], verts[i0]);
            }
        }

        vertexCount = static_cast<GLsizei>(m_Vertices.size() / 8);
    }

    void PushTriangle(const VertexData& v0, const VertexData& v1, const VertexData& v2)
    {
        auto pushVertex = [this](const glm::vec3& v, const glm::vec2& uv)
            {
                glm::vec3 n = glm::normalize(v);
                m_Vertices.push_back(v.x);
                m_Vertices.push_back(v.y);
                m_Vertices.push_back(v.z);
                m_Vertices.push_back(n.x);
                m_Vertices.push_back(n.y);
                m_Vertices.push_back(n.z);
                m_Vertices.push_back(uv.x);
                m_Vertices.push_back(uv.y);
            };

        pushVertex(v0.pos, v0.uv);
        pushVertex(v1.pos, v1.uv);
        pushVertex(v2.pos, v2.uv);
    }

    void SetupBuffers() override
    {

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);

        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, m_Vertices.size() * sizeof(float), m_Vertices.data(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);
    }

    std::vector<float> m_Vertices;

    int m_Slices;
    int m_Stacks;
    float m_Radius;
};