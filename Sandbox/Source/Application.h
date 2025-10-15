#pragma once
#include "Window.h"

class Application
{
public:
    Application()  : camera(glm::vec3(0.0f, 0.0f, 3.0f)), gui(m_Window.GetNative()) { Init(); }
    virtual ~Application();

    void Run()
    {

        while (!m_Window.ShouldClose())
        {
            float currentFrame = static_cast<float>(m_Window.GetTime());
            m_DeltaTime = currentFrame - m_LastFrame;
            m_LastFrame = currentFrame;
            
            Update(m_DeltaTime);
        }

        Shutdown();
    }

private:
    virtual void Init() = 0;
    virtual void Update(float dt) = 0;
    virtual void Shutdown() = 0;

    // Callback wrappers
   

protected:

    Window m_Window;
    Gui gui;
    Camera camera;

    float m_LastFrame = 0.0f;
    float m_DeltaTime = 0.0f;
    bool m_FirstMouse = true;
    float m_LastX = 0.0f;
    float m_LastY = 0.0f;
};