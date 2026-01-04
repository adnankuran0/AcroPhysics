#pragma once
#include <memory>
#include "Scene.h"
#include "Window.h"
#include "DebugSettings.h"
#include "Render/Shader.h"
#include "DebugRenderer/DebugRendererGL.h"
#include "Render/Texture.h"
#include "Render/ShadowManager.h"
#include "Render/Skybox.h"
#include "Gui.h"
#include "Render/InstancedRenderer.h"
#include "Camera.h"
#include "Render/Cube.h"
#include "Render/Sphere.h"

class Application
{
public:
	Application(const char* exePath);
	void Run();
	~Application();

private:
	// callbacks
	static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
	static void mouse_callback(GLFWwindow* window, double xpos, double ypos);
	static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
	static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);

	void OnResize(int width, int height);
	void OnMouseMove(GLFWwindow* window, double xpos, double ypos);
	void OnScroll(double xoffset, double yoffset);
	void OnKey(int key, int scancode, int action, int mods);

	void processInput(GLFWwindow* window);
private:
	float SCR_WIDTH = 1280;
	float SCR_HEIGHT = 720;
	Scene scene;
	Window window;
	DebugSettings settings;
	Shader shader;
	Shader skyboxShader;
	DebugRendererGL debugRendererGL;
	Texture crateTexture;
	ShadowManager shadowManager;
	Skybox skybox;
	Gui gui;
	InstancedRenderer cubeRenderer;
	InstancedRenderer sphereRenderer;
	Camera camera;
	Cube cubeMesh;
	Sphere sphereMesh;
	std::unique_ptr<Acro::World> world;
	bool firstMouse = true;
	float lastX = 1280.0f / 2.0f;
	float lastY = 720.0f / 2.0f;
};