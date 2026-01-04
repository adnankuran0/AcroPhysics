#include "Gui.h"

Gui::Gui(GLFWwindow* window)
{
    IMGUI_CHECKVERSION();
    ImGui::SetCurrentContext(ImGui::CreateContext());
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 460");
}

void Gui::NewFrame()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void Gui::Update(DebugSettings& settings)
{
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Checkbox("Simulate physics", &settings.stepPhysics);
    ImGui::Checkbox("Debug draw", &settings.debugDraw);
    ImGui::Checkbox("Draw Contact Points", &settings.drawContactPoints);
    ImGui::Checkbox("Draw AABBs", &settings.drawAABBs);
    ImGui::Checkbox("Draw shapes", &settings.drawShapes);
    ImGui::Checkbox("Pause", &settings.pause);
}

void Gui::Render()
{
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Gui::Shutdown()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}