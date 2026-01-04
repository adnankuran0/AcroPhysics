#include "ShadowManager.h"
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/matrix_clip_space.hpp>

void ShadowManager::Init(unsigned int shadowRes) 
{
	m_ShadowRes = shadowRes;

	glGenFramebuffers(1, &m_ShadowFBO);

	glGenTextures(1, &m_ShadowMap);
	glBindTexture(GL_TEXTURE_2D, m_ShadowMap);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, m_ShadowRes,m_ShadowRes,0,GL_DEPTH_COMPONENT,GL_FLOAT,nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
	float borderColor[] = { 1.0f,1.0f,1.0f,1.0f };
	glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
	glBindFramebuffer(GL_FRAMEBUFFER, m_ShadowFBO);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, m_ShadowMap, 0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	m_DepthShader.Init("Depth.vs", "Depth.fs");

	glm::vec3 lightDir = glm::normalize(glm::vec3(-0.2f, -1.0f, -0.3f));
	glm::vec3 lightPos = -lightDir * 10.0f;
	glm::mat4 lightView = glm::lookAt(
		lightPos,
		glm::vec3(0.0f),
		glm::vec3(0, 1, 0)
	);
	glm::mat4 lightProj = glm::ortho(-15.f, 15.f, -15.f, 15.f, 0.1f, 50.0f);
	m_LightSpaceMat = lightProj * lightView;

	m_DepthShader.use();
	m_DepthShader.setMat4("lightSpace", m_LightSpaceMat);
}