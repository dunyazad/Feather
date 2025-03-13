#include<Core/Component/GUIComponent/StatusPanel.h>

StatusPanel::StatusPanel(ComponentID id)
	: GUIComponentBase(id), fpsHistory(historySize, 60.0f)
{
	AddEventHandler(EventType::MousePosition, [&](const Event& event) {
        mouseX = (ui32)event.parameters.mousePosition.xpos;
        mouseY = (ui32)event.parameters.mousePosition.ypos;
		});
}

StatusPanel::~StatusPanel()
{
}

void StatusPanel::Render()
{
    float fps = ImGui::GetIO().Framerate;
    accumulatedFPS += fps;
    frameCount++;

    // Update the graph every few frames
    if (frameCount >= updateRate)
    {
        fpsHistory[historyOffset] = accumulatedFPS / updateRate;
        historyOffset = (historyOffset + 1) % historySize;
        accumulatedFPS = 0.0f;
        frameCount = 0;
    }

    // Position overlay on the top-left
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.7f); // Semi-transparent background

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 12.0f); // Rounded corners
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.3f, 0.3f, 0.6f, 0.8f)); // Green background

    if (ImGui::Begin("Performance Overlay", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove))
    {
        //float memoryUsage = GetMemoryUsageMB();
        //ImGui::Text("Mem: %.1fmb", memoryUsage);

        // FPS Display with White Text
        ImGui::Text("FPS: %.1f", fps);

        // Graph (Mini FPS history)
        ImGui::PlotLines("##FPSGraph", fpsHistory.data(), historySize, historyOffset, nullptr, 0.0f, 120.0f, ImVec2(200, 80));

        ImGui::Text("mouse : %4d, %4d", mouseX, mouseY);
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}