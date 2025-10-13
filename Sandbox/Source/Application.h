#pragma once
#include "Window.h"

class Application
{
public:
    Application();
    virtual ~Application();

    void Run()
    {
        Init();

        while (!m_Window.ShouldClose())
        {
            float currentFrame = static_cast<float>(m_Window.GetTime());
            m_DeltaTime = currentFrame - m_LastFrame;
            m_LastFrame = currentFrame;

            ProcessInput(m_DeltaTime);
            Update(m_DeltaTime);
            Render();
            RenderGui();
        }

        Shutdown();
    }

private:
    virtual void Init() = 0;
    virtual void Update(float dt) = 0;
    virtual void Render() = 0;
    virtual void RenderGui() = 0;
    virtual void ProcessInput(float dt) = 0;
    virtual void Shutdown() = 0;

    // Callback wrappers
   

protected:
    virtual void OnResize(int width, int height) = 0;
    virtual void OnMouseMove(double x, double y) = 0;
    virtual void OnScroll(double xoffset, double yoffset) = 0;
    virtual void OnKey(int key, int scancode, int action, int mods) = 0;

    Window m_Window;

    float m_LastFrame = 0.0f;
    float m_DeltaTime = 0.0f;
    bool m_FirstMouse = true;
    float m_LastX = 0.0f;
    float m_LastY = 0.0f;
};