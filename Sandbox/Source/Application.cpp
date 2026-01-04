#include "Application.h"

#include "Timer.h"

using namespace Acro::Math;

Application::Application(const char* exePath) : camera(glm::vec3(0.0f, 5.0f, 20.0f))
{
    Path::Init(exePath);

    window.Init("Acro Sandbox", SCR_WIDTH, SCR_HEIGHT);
    window.SetFramebufferSizeCallback(framebuffer_size_callback);
    window.SetCursorPosCallback(mouse_callback);
    window.SetScrollCallback(scroll_callback);
    window.SetKeyCallback(key_callback);

    gui.Init(window.GetNative());
    
    shader.Init("Phong.vs", "Phong.fs");
    skyboxShader.Init("Skybox.vs", "Skybox.fs");

    skybox.Init();
    
    cubeMesh.Init();
    sphereMesh.Init();

    cubeRenderer.Init(cubeMesh, 100);
    sphereRenderer.Init(sphereMesh, 100);

    shadowManager.Init();
    
    crateTexture.Init("crate.png");
    crateTexture.Bind(0);

    shader.use();
    shader.setMat4("lightSpace", shadowManager.GetLightSpaceMat());
    shader.setInt("shadowMap", 1);
   
    world = std::make_unique<Acro::World>(Acro::Math::Vector3(0.0f, -9.8f, 0.0f), 144, 8);

    scene.Init(*world);

    debugRendererGL.Init(
        "Line.vs", "Line.fs",
        "Point.vs", "Point.fs");
}

void Application::Run()
{
    while (!window.ShouldClose())
    {
       

        Timer::Update(glfwGetTime());

        processInput(window.GetNative());

        gui.NewFrame();
        gui.Update(settings);

        world->SetPaused(!settings.stepPhysics);
        world->GetDebugRenderer().drawContactPoints = settings.drawContactPoints;
        world->GetDebugRenderer().drawAABBs = settings.drawAABBs;
        world->GetDebugRenderer().drawShapes = settings.drawShapes;

        if (ImGui::Button("Reset"))
        {
            settings.stepPhysics = false;
            world->SetPaused(true);
            scene.Reset();
        }

        if (!settings.pause)
            scene.Update(glfwGetTime());

        world->Step(Timer::deltaTime);

        window.Clear();

        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 proj = camera.GetProjectionMatrix();

        skybox.Draw(skyboxShader, view, proj);

        shader.use();

        sphereRenderer.Begin();
        cubeRenderer.Begin();

        glm::mat4 groundTransform(1.0f);
        groundTransform = glm::translate(groundTransform, static_cast<glm::vec3>(scene.GetGround().GetPosition()));
        groundTransform *= glm::toMat4(static_cast<glm::quat>(scene.GetGround().GetOrientation()));
        groundTransform = glm::scale(groundTransform, glm::vec3(20.0f, 0.2f, 20.0f));
        cubeRenderer.Submit(groundTransform);


        for (auto& b : scene.GetBodies())
        {
            glm::mat4 model = glm::mat4(1.0f);
            model = glm::translate(model, static_cast<glm::vec3>(b.GetPosition()));
            model *= glm::toMat4(static_cast<glm::quat>(b.GetOrientation()));

            if (b.GetShapeType() == Acro::ShapeType::Box)
                cubeRenderer.Submit(model);
            else
                sphereRenderer.Submit(model);
        }

        shadowManager.GetDepthShader().use();
        cubeRenderer.DrawDepth(shadowManager.GetDepthShader(), shadowManager.GetLightSpaceMat(),
            shadowManager.GetFramebuffer(), shadowManager.GetShadowResolution());
        sphereRenderer.DrawDepth(shadowManager.GetDepthShader(), shadowManager.GetLightSpaceMat(),
            shadowManager.GetFramebuffer(), shadowManager.GetShadowResolution());
        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, shadowManager.GetShadowMap());
        shader.use();
        shader.setInt("shadowMap", 1);
        cubeRenderer.Draw(shader, view, proj, camera.Position);
        sphereRenderer.Draw(shader, view, proj, camera.Position);


        gui.Render();

        if (settings.debugDraw)
            debugRendererGL.Render(world->GetDebugRenderer(), view, proj, Timer::deltaTime);
        
        /* Swap front and back buffers */
        window.SwapBuffers();

        GLenum err = glGetError();
        if (err)
            std::cout << "OpenGL error: " << err << "\n";

        /* Poll for and process events */
        window.PollEvents();
    }
}

Application::~Application()
{
    gui.Shutdown();
    window.Shutdown();
}

void Application::framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) app->OnResize(width, height);
}

void Application::mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) app->OnMouseMove(window,xpos, ypos);
}

void Application::scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) app->OnScroll(xoffset, yoffset);
}

void Application::key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    if (app) app->OnKey(key, scancode, action, mods);
}

void Application::OnResize(int width, int height)
{
    SCR_WIDTH = width;
    SCR_HEIGHT = height;
}

void Application::OnMouseMove(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (glfwGetInputMode(window, GLFW_CURSOR) != GLFW_CURSOR_DISABLED)
    {
        return;
    }

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; 

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void Application::OnScroll(double xoffset, double yoffset)
{
    std::cout << "Yarraaa\n";
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

void Application::OnKey(int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
    {
        scene.GetBodies()[13].ApplyForce(Acro::Math::Vector3(0.0f, 800.0f, 0.0f));
    }
}

void Application::processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);


    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS)
    {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        double xpos, ypos;
        glfwGetCursorPos(window, &xpos, &ypos);

        if (firstMouse)
        {
            lastX = static_cast<float>(xpos);
            lastY = static_cast<float>(ypos);
            firstMouse = false;
        }

        float xoffset = static_cast<float>(xpos) - lastX;
        float yoffset = lastY - static_cast<float>(ypos); 

        lastX = static_cast<float>(xpos);
        lastY = static_cast<float>(ypos);

        camera.ProcessMouseMovement(xoffset, yoffset);

        Vector3 forward = Vector3(camera.Front.x, 0, camera.Front.z);
        Vector3 right = Vector3(camera.Right.x, 0, camera.Right.z);

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(FORWARD, Timer::deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(BACKWARD, Timer::deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(LEFT, Timer::deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(RIGHT, Timer::deltaTime);
    }
    else
    {
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
        firstMouse = true;

        Vector3 forward = Vector3(camera.Front.x, 0, camera.Front.z);
        Vector3 right = Vector3(camera.Right.x, 0, camera.Right.z);

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) scene.GetBodies()[13].ApplyForce(forward);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) scene.GetBodies()[13].ApplyForce(-forward);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) scene.GetBodies()[13].ApplyForce(right);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) scene.GetBodies()[13].ApplyForce(-right);
    }

    float scrollAmount = ImGui::GetIO().MouseWheel;

    if (scrollAmount != 0.0f)
    {
        if (glfwGetInputMode(window, GLFW_CURSOR) == GLFW_CURSOR_DISABLED)
        {
            camera.ProcessMouseScroll(scrollAmount);
        }
    }
}

