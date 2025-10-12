#include <glad/gl.h>

#include "Gui.h"

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <Shader.h>
#include <Camera.h>
#include "Acro.h"
#include <iostream>
#include "DebugCube.h"
#include <memory>
#define STB_IMAGE_IMPLEMENTATION
#include "Skybox.h"
#include "imgui.h"

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


Acro::Rigidbody body(nullptr,nullptr,Acro::Core::BodyHandle{});

int main(void)
{
    bool stepPhysics = false;

    GLFWwindow* window;

    /* Initialize the library */
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Acro Sandbox", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, key_callback);


    int version = gladLoadGL(glfwGetProcAddress);
    if (version == 0) {
        printf("Failed to initialize OpenGL context\n");
        return -1;
    }

    printf("Loaded OpenGL %d.%d\n", GLAD_VERSION_MAJOR(version), GLAD_VERSION_MINOR(version));

    Gui gui(window);

    glEnable(GL_DEPTH_TEST);

    Shader shader("D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Phong.vs", "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Phong.fs");
    Shader skyboxShader("D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Skybox.vs", "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Shaders\\Skybox.fs");

    std::vector<std::string> faces
    {
        "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Textures\\Skybox\\right.jpg",
        "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Textures\\Skybox\\left.jpg",
        "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Textures\\Skybox\\top.jpg",
        "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Textures\\Skybox\\bottom.jpg",
        "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Textures\\Skybox\\front.jpg",
        "D:\\GitHub\\AcroPhysics\\Sandbox\\Source\\Textures\\Skybox\\back.jpg"
    };

    Skybox skybox(faces);

    DebugCube cube;

    Acro::World world(Acro::Math::Vector3(0.0f,-9.8f,0.0f),60,8);

    body = world.CreateBody();

    Acro::BoxShape boxShape = world.CreateBoxShape();
    Acro::SphereShape sphereShape = world.CreateSphereShape(5.0f);

    Acro::Rigidbody body1 = world.CreateBody();
    Acro::Rigidbody body2 = world.CreateBody();
    Acro::Rigidbody body3 = world.CreateBody();

    body1.AttachShape(boxShape);
    body2.AttachShape(boxShape);
    body3.AttachShape(sphereShape);

    body1.Destroy();
    body2.Destroy();
    body3.Destroy();
    

    while (!glfwWindowShouldClose(window))
    {
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        gui.NewFrame();

        ImGui::Checkbox("Simulate physics", &stepPhysics);

        if(stepPhysics)
            world.Step(deltaTime);

        processInput(window);

        glClearColor(0.0, 0.0, 0.0, 1.0);
        /* Render here */
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        skybox.Draw(skyboxShader, camera.GetViewMatrix(), camera.GetProjectionMatrix());

        glm::vec3 pos = body.GetPosition();
        glm::quat rot = body.GetOrientation();
        //std::cout << pos.x << "," << pos.y << "," << pos.z << "\n";

        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 proj = camera.GetProjectionMatrix();

        cube.SetPosition(pos);
        cube.SetRotation(rot);
        cube.Draw(shader, view, proj, camera.Position);

        gui.Render();

        /* Swap front and back buffers */
        glfwSwapBuffers(window);

        GLenum err = glGetError();
        if(err)
            std::cout << "OpenGL error: " << err << "\n";

        /* Poll for and process events */
        glfwPollEvents();
    }

    gui.Shutdown();

    glfwTerminate();
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