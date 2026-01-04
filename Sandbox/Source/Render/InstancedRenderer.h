#pragma once
#include "Mesh.h"
#include <vector>
#include <glm/glm.hpp>
#include "Render/Shader.h"

class InstancedRenderer
{
public:
    InstancedRenderer() = default;
    InstancedRenderer(const Mesh& mesh, unsigned int maxInstances) { Init(mesh, maxInstances); }
    void Init(const Mesh& mesh, unsigned int maxInstances);
    void Begin();
    void Submit(const glm::mat4& transform);
    void Draw(const Shader& shader, const glm::mat4& view, const glm::mat4& proj, const glm::vec3& cameraPos) const;

private:
    const Mesh* m_Mesh = nullptr;

    std::vector<glm::mat4> m_Transforms;
    GLuint m_InstanceVBO = 0;
    unsigned int m_MaxInstances = 0;
};
