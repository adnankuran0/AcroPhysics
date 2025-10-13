#pragma once
#include <glad/gl.h>
#include "GLFW/glfw3.h"

class Window
{
public:
	int Init(const char* windowName, unsigned int screenWidth, unsigned int screenHeight, bool vsync = false);
	void Shutdown() { glfwTerminate(); }
	inline GLFWwindow* GetNative() noexcept { return m_NativeWindow; }
	inline void Clear(float r = 0.0f, float g = 0.0f, float b = 0.0f, float a = 1.0f) noexcept
	{
		glClearColor(r, g, b, a);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	inline double GetTime() noexcept { return glfwGetTime(); }
	inline void SetFramebufferSizeCallback(GLFWframebuffersizefun callback) { glfwSetFramebufferSizeCallback(m_NativeWindow, callback); }
	inline void SetCursorPosCallback(GLFWcursorposfun callback) { glfwSetCursorPosCallback(m_NativeWindow, callback); }
	inline void SetScrollCallback(GLFWscrollfun callback) { glfwSetScrollCallback(m_NativeWindow, callback); }
	inline void SetKeyCallback(GLFWkeyfun callback) { glfwSetKeyCallback(m_NativeWindow, callback); }
	inline bool ShouldClose() { return glfwWindowShouldClose(m_NativeWindow); }
	inline void SwapBuffers() { glfwSwapBuffers(m_NativeWindow); }
	inline void PollEvents() { glfwPollEvents(); }

private:
	GLFWwindow* m_NativeWindow;
};