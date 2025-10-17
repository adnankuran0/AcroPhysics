#include <Window.h>
#include "Gui.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <Shader.h>
#include <Camera.h>
#include "Acro.h"
#include "Cube.h"
#include <memory>
#define STB_IMAGE_IMPLEMENTATION
#include "Skybox.h"
#include "imgui.h"
#include "DebugRenderer/DebugRendererGL.h"
#include "Sphere.h"


using namespace Acro::Math;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void processInput(GLFWwindow* window);

#define SCR_WIDTH 1280
#define SCR_HEIGHT 720

Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

float deltaTime = 0.0f;
float lastFrame = 0.0f;


Acro::Rigidbody body(nullptr,nullptr,nullptr,Acro::Core::BodyHandle{});

int main(void)
{
    bool stepPhysics = false;
    bool debugDraw = true;
    bool drawAABBs = false;
    bool drawShapes = false;


    /* Initialize the library */
    Window window;
    window.Init("Acro Sandbox", SCR_WIDTH, SCR_HEIGHT);
    window.SetFramebufferSizeCallback(framebuffer_size_callback);
    window.SetCursorPosCallback(mouse_callback);
    window.SetScrollCallback(scroll_callback);
    window.SetKeyCallback(key_callback);

    Gui gui(window.GetNative());

    Shader shader("D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Phong.vs", "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Phong.fs");
    Shader skyboxShader("D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Skybox.vs", "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Skybox.fs");

    Skybox skybox;
    skybox.Init();

    Cube cube;
    Cube cube2;
    Sphere sphere;

    Acro::World world(Acro::Math::Vector3(0.0f,-9.8f,0.0f),60,8);

    body = world.CreateBody();

    Acro::BoxShape boxShape = world.CreateBoxShape(Acro::Math::Vector3(0.5f, 0.5f, 0.5f));
    Acro::BoxShape boxShape2 = world.CreateBoxShape(Acro::Math::Vector3(1.5f, 1.5f, 1.5f));

    Acro::BodyDescription desc;
    desc.position = Vector3(2.0, 0.0, 0.0);
    desc.mass = 0.0f; // static body
    Acro::Rigidbody body2 = world.CreateBody(desc);

    Acro::BodyDescription sphereDesc;
    sphereDesc.position = Vector3(-2.0, 0.0, 0.0);
    Acro::Rigidbody sphereBody = world.CreateBody(sphereDesc);
    Acro::SphereShape sphereShape = world.CreateSphereShape(0.5f);
    sphereBody.AttachShape(sphereShape);

    body.AttachShape(boxShape);
    body.AttachShape(boxShape);
    body2.AttachShape(boxShape);


    DebugRendererGL debugRendererGL;
    debugRendererGL.Init("D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Line.vs", "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Line.fs",
        "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Point.vs", "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Point.fs");

    while (!window.ShouldClose())
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        gui.NewFrame();
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        if(ImGui::Checkbox("Simulate physics", &stepPhysics))
        {
            world.SetPaused(!stepPhysics);
        }
        ImGui::Checkbox("Debug draw", &debugDraw);
        if (ImGui::Checkbox("Draw AABBs", &drawAABBs))
        {
            world.GetDebugRenderer().drawAABBs = drawAABBs;
        }
        if (ImGui::Checkbox("Draw shapes", &drawShapes))
        {
            world.GetDebugRenderer().drawShapes = drawShapes;
        }
            
        world.Step(deltaTime);

        processInput(window.GetNative());

        window.Clear();

        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 proj = camera.GetProjectionMatrix();

        skybox.Draw(skyboxShader, view, proj);

        body.SetOrientation(Quaternion(Vector3(1.0, 0.5, 0.0), 45.0f + static_cast<float>(glfwGetTime())));

        cube.Draw(shader, body,view, proj, camera.Position);
        cube2.Draw(shader, body2,view, proj, camera.Position);
        sphere.Draw(shader, sphereBody, view, proj, camera.Position);

        gui.Render();

        if (debugDraw)
            debugRendererGL.Render(world.GetDebugRenderer(), view, proj);

        world.GetDebugRenderer().Clear();

        /* Swap front and back buffers */
        window.SwapBuffers();

        GLenum err = glGetError();
        if(err)
            std::cout << "OpenGL error: " << err << "\n";

        /* Poll for and process events */
        window.PollEvents();
    }

    gui.Shutdown();
    window.Shutdown();
    return 0;
}

void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}


void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    if (!glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2))
    {
        firstMouse = true; 
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        return;
    }

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        body.ApplyForce(Acro::Math::Vector3(0.0f, 400.0f, 0.0f));
    }
}