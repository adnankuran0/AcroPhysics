#include <Window.h>
#include "Gui.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <Render/Shader.h>
#include <Camera.h>
#include "Acro.h"
#include "Render/Cube.h"
#include <memory>
#define STB_IMAGE_IMPLEMENTATION
#include "Render/Skybox.h"
#include "imgui.h"
#include "DebugRenderer/DebugRendererGL.h"
#include "Render/Sphere.h"
#include "Render/Texture.h"
#include "Render/InstancedRenderer.h"
#include "Path.h"
#include "DebugSettings.h"
#include "Timer.h"


using namespace Acro::Math;
using namespace Acro::Physics;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void processInput(GLFWwindow* window);

#define SCR_WIDTH 1280
#define SCR_HEIGHT 720

static Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));



static struct BodyInitialState {
    Vector3 position;
    Quaternion orientation;
};
static std::vector<BodyInitialState> initialStates;
static bool requestReset = false;
static std::vector<Acro::Rigidbody> gridBodies;

int main(void)
{
   
    DebugSettings settings;

    /* Initialize the library */
    Window window;
    window.Init("Acro Sandbox", SCR_WIDTH, SCR_HEIGHT);
    window.SetFramebufferSizeCallback(framebuffer_size_callback);
    window.SetCursorPosCallback(mouse_callback);
    window.SetScrollCallback(scroll_callback);
    window.SetKeyCallback(key_callback);

    Gui gui(window.GetNative());

    Shader shader("Phong.vs", "Phong.fs");
    Shader skyboxShader("Skybox.vs", "Skybox.fs");

    Skybox skybox;
    skybox.Init();

    Texture crateTexture("crate.png");
    crateTexture.Bind(0);

    Cube cubeMesh;
    Sphere sphereMesh;
    InstancedRenderer cubeRenderer(cubeMesh,100);
    InstancedRenderer sphereRenderer(sphereMesh,100);

    Acro::World world = Acro::World(Acro::Math::Vector3(0.0f,-9.8f,0.0f),144,8);

    Acro::BodyDescription groundDesc;
    groundDesc.position = Vector3(0.0f, -2.0f, 0.0f);
    groundDesc.mass = 0.0f;

    Acro::Rigidbody groundBody = world.CreateBody(groundDesc);
    Acro::BoxShape groundShape = world.CreateBoxShape(Vector3(10.0f, 0.1f, 10.0f));
    groundBody.AttachShape(groundShape);

    Acro::BoxShape cubeShape = world.CreateBoxShape(Vector3(0.5f, 0.5f, 0.5f));
    Acro::SphereShape sphereShape = world.CreateSphereShape(0.5f);

    const int rows = 6;
    const int cols = 6;
    const float spacing = 2.0f;

    for (int x = 0; x < cols; x++)
    {
        for (int z = 0; z < rows; z++)
        {
            float posX = -6.0f + x * spacing;
            float posZ = -4.0f + z * spacing;
            float height = 1.0f + (x + z) * 0.6f;

            Acro::BodyDescription d;
            d.position = Vector3(posX, height, posZ);
            d.mass = 1.0f;

            Acro::Rigidbody body = world.CreateBody(d);

            //if ((x + z) % 2 == 0)
                //body.AttachShape(cubeShape);
            //else
            body.AttachShape(cubeShape);

            gridBodies.push_back(body);

            initialStates.push_back({ d.position, d.orientation });
        }
    }

    DebugRendererGL debugRendererGL;
    debugRendererGL.Init("Line.vs", "Line.fs",
        "Point.vs","Point.fs");


    while (!window.ShouldClose())
    {
        Timer::Update(glfwGetTime());

        processInput(window.GetNative());

        gui.NewFrame();
        gui.Update(settings);
        
        world.SetPaused(!settings.stepPhysics);
        world.GetDebugRenderer().drawContactPoints  = settings.drawContactPoints;
        world.GetDebugRenderer().drawAABBs = settings.drawAABBs;
        world.GetDebugRenderer().drawShapes = settings.drawShapes;

        if (ImGui::Button("Reset") || requestReset)
        {
            requestReset = false;
            settings.stepPhysics = false;
            world.SetPaused(true);

            for (size_t i = 0; i < gridBodies.size(); i++)
            {
                gridBodies[i].SetPosition(initialStates[i].position);
                gridBodies[i].SetOrientation(initialStates[i].orientation);
                gridBodies[i].SetLinearVelocity(Vector3(0.0f, 0.0f, 0.0f));
                gridBodies[i].SetAngularVelocity(Vector3(0.0f, 0.0f, 0.0f));
            }

            groundBody.SetOrientation(Quaternion(Vector3(1.0f, 0.0f, 0.0f), 0.0f));
        }

        if(!settings.pause)
            groundBody.SetOrientation(Quaternion(Vector3(1.0f, 0.0f, 0.0f),glfwGetTime()*0.5f));
        
        world.Step(Timer::deltaTime);

        window.Clear();

        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 proj = camera.GetProjectionMatrix();

        skybox.Draw(skyboxShader, view, proj);
       
        shader.use();

        // render ground
        cubeRenderer.Begin();
        glm::mat4 groundTransform(1.0f);
        groundTransform = glm::translate(groundTransform, static_cast<glm::vec3>(groundBody.GetPosition()));
        groundTransform *= glm::toMat4(static_cast<glm::quat>(groundBody.GetOrientation()));
        groundTransform = glm::scale(groundTransform, glm::vec3(20.0f, 0.2f, 20.0f));
        cubeRenderer.Submit(groundTransform);
        cubeRenderer.Draw(shader, view, proj, camera.Position);

        cubeRenderer.Begin();
        sphereRenderer.Begin();

        for (auto& b : gridBodies)
        {
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, static_cast<glm::vec3>(b.GetPosition()));
            model *= glm::toMat4(static_cast<glm::quat>(b.GetOrientation()));

            if (b.GetShapeType() == Acro::ShapeType::Box)
                cubeRenderer.Submit(model);
            else
                sphereRenderer.Submit(model);
        }

        crateTexture.Bind(0);
        cubeRenderer.Draw(shader, view, proj, camera.Position);
        sphereRenderer.Draw(shader, view, proj, camera.Position);

        gui.Render();

        if (settings.debugDraw)
            debugRendererGL.Render(world.GetDebugRenderer(), view, proj, Timer::deltaTime);

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
        camera.ProcessKeyboard(FORWARD, Timer::deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, Timer::deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, Timer::deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, Timer::deltaTime);

    Vector3 forward = Vector3(camera.Front.x, 0, camera.Front.z);
    Vector3 right = Vector3(camera.Right.x, 0, camera.Right.z);
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        gridBodies[13].ApplyForce(forward);
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        gridBodies[13].ApplyForce(-forward);
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        gridBodies[13].ApplyForce(right);
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        gridBodies[13].ApplyForce(-right);
}   

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    camera.ProcessMouseMovement(window,xposIn, yposIn);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
   
}
