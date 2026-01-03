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
#include <Texture.h>


using namespace Acro::Math;
using namespace Acro::Physics;

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

struct BodyInitialState {
    Vector3 position;
    Quaternion orientation;
};
std::vector<BodyInitialState> initialStates;
bool requestReset = false;
std::vector<Acro::Rigidbody> gridBodies;

std::unique_ptr<Acro::World> world;

int main(void)
{
    bool stepPhysics = false;
    bool debugDraw = true;
    bool drawContactPoints = false;
    bool drawAABBs = false;
    bool drawShapes = false;
    bool pause = true;


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

    Texture crateTexture("D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Textures\\crate.png");

    Cube cube;
    Cube cube2;
    Sphere sphere;
    Sphere sphere2;

    world = std::make_unique<Acro::World>(Acro::Math::Vector3(0.0f,-9.8f,0.0f),144,8);

    Acro::BodyDescription groundDesc;
    groundDesc.position = Vector3(0.0f, -2.0f, 0.0f);
    //groundDesc.orientation = Quaternion(Vector3(1.0, 0.0, 0.0f), 0.25f);
    groundDesc.mass = 0.0f;

    Acro::Rigidbody groundBody = world->CreateBody(groundDesc);
    Acro::BoxShape groundShape = world->CreateBoxShape(Vector3(10.0f, 0.1f, 10.0f));
    groundBody.AttachShape(groundShape);

    Acro::BoxShape cubeShape = world->CreateBoxShape(Vector3(0.5f, 0.5f, 0.5f));
    Acro::SphereShape sphereShape = world->CreateSphereShape(0.5f);
 

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
            //d.orientation = Quaternion(Vector3(0.5f, 0.5f, 0.0f), 45.f);
            d.mass = 1.0f;

            Acro::Rigidbody body = world->CreateBody(d);

            //if ((x + z) % 2 == 0)
                //body.AttachShape(cubeShape);
            //else
            body.AttachShape(sphereShape);

            gridBodies.push_back(body);

            initialStates.push_back({ d.position, d.orientation });
        }
    }

    DebugRendererGL debugRendererGL;
    debugRendererGL.Init("D:\\Github\\AcroPhysics\\Sandbox\\Source\\Shaders\\Line.vs", "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Line.fs",
        "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Point.vs","D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Point.fs");


    while (!window.ShouldClose())
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        gui.NewFrame();
        ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
        if(ImGui::Checkbox("Simulate physics", &stepPhysics))
        {
            world->SetPaused(!stepPhysics);
        }
        ImGui::Checkbox("Debug draw", &debugDraw);
        if (ImGui::Checkbox("Draw Contact Points", &drawContactPoints))
        {
            world->GetDebugRenderer().drawContactPoints  = drawContactPoints;
        }
        if (ImGui::Checkbox("Draw AABBs", &drawAABBs))
        {
            world->GetDebugRenderer().drawAABBs = drawAABBs;
        }
        if (ImGui::Checkbox("Draw shapes", &drawShapes))
        {
            world->GetDebugRenderer().drawShapes = drawShapes;
        }

        ImGui::Checkbox("Pause", &pause);

        if (ImGui::Button("Reset") || requestReset)
        {
            requestReset = false;
            stepPhysics = false;
            world->SetPaused(true);

            for (size_t i = 0; i < gridBodies.size(); i++)
            {
                gridBodies[i].SetPosition(initialStates[i].position);
                gridBodies[i].SetOrientation(initialStates[i].orientation);
                gridBodies[i].SetLinearVelocity(Vector3(0.0f, 0.0f, 0.0f));
                gridBodies[i].SetAngularVelocity(Vector3(0.0f, 0.0f, 0.0f));
            }

            groundBody.SetOrientation(Quaternion(Vector3(1.0f, 0.0f, 0.0f), 0.0f));
        }

        if(!pause)
            groundBody.SetOrientation(Quaternion(Vector3(1.0f, 0.0f, 0.0f),glfwGetTime()*0.5f));
        
        //camera.Position = gridBodies[13].GetPosition();

        world->Step(deltaTime);

        processInput(window.GetNative());

        window.Clear();

        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 proj = camera.GetProjectionMatrix();

        skybox.Draw(skyboxShader, view, proj);
        
        
        cube.SetScale(glm::vec3(20.0f, 0.2f, 20.0f));
        cube.Draw(shader, groundBody, view, proj, camera.Position);
        cube.SetScale(glm::vec3(1.0f));

        crateTexture.Bind(0);
        for (auto& b : gridBodies)
        {
            if (b.GetShapeType() == Acro::ShapeType::Box)
                cube.Draw(shader, b, view, proj, camera.Position);
            else
                sphere.Draw(shader, b, view, proj, camera.Position);
        }
       

        gui.Render();

        

        if (debugDraw)
            debugRendererGL.Render(world->GetDebugRenderer(), view, proj,deltaTime);

        

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
    }
}
