#pragma once
#include "glad/gl.h"
#include "Shader.h"

class ShadowManager
{
public:
	void Init(unsigned int shadowRes = 1024);
	inline GLuint GetFramebuffer() { return m_ShadowFBO; }
	inline GLuint GetShadowMap() { return m_ShadowMap; }
	inline unsigned int GetShadowResolution() { return m_ShadowRes; }
	inline const Shader& GetDepthShader() { return m_DepthShader; }
	inline const glm::mat4& GetLightSpaceMat() { return m_LightSpaceMat; }
private:

	Shader m_DepthShader{};
	GLuint m_ShadowFBO{};
	GLuint m_ShadowMap{};
	glm::mat4 m_LightSpaceMat{};
	unsigned int m_ShadowRes = 1024;
};