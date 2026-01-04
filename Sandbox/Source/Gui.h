#pragma once

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "DebugSettings.h"

class Gui
{
public:
	Gui() = default;
	Gui(GLFWwindow* window);
	void Init(GLFWwindow* window);
	void NewFrame();
	void Update(DebugSettings& settings);
	void Render();
	void Shutdown();
};