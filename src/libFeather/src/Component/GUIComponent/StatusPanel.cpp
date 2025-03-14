#include<Component/GUIComponent/StatusPanel.h>

StatusPanel::StatusPanel()
	: RegisterDerivation<StatusPanel, GUIComponentBase>(), fpsHistory(historySize, 60.0f)
{
	AddEventHandler(EventType::MousePosition, [&](const Event& event) {
        mouseX = (ui32)event.parameters.mousePosition.xpos;
        mouseY = (ui32)event.parameters.mousePosition.ypos;
		});
}

StatusPanel::~StatusPanel()
{
}

void ToggleButton(const char* str_id, bool* v)
{
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float height = ImGui::GetFrameHeight();
    float width = height * 1.55f;
    float radius = height * 0.50f;

    if (ImGui::InvisibleButton(str_id, ImVec2(width, height)))
    {
        *v = !*v;
        if (true == *v)
        {
            glfwSwapInterval(1);
        }
        else if (false == *v)
        {
            glfwSwapInterval(0);
        }
    }
    ImU32 col_bg;
    if (ImGui::IsItemHovered())
        col_bg = *v ? IM_COL32(145 + 20, 211, 68 + 20, 255) : IM_COL32(218 - 20, 218 - 20, 218 - 20, 255);
    else
        col_bg = *v ? IM_COL32(145, 211, 68, 255) : IM_COL32(218, 218, 218, 255);

    draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height - 5), col_bg, (height - 5) * 0.5f);
    draw_list->AddCircleFilled(ImVec2(*v ? (p.x + width - radius) : (p.x + radius), p.y + radius - 2.5f), (radius - 2.5f) - 1.5f, IM_COL32(255, 255, 255, 255));
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
        ImGui::Text("V-Sync");
        ImGui::SameLine();
        ToggleButton("V-Sync", &vSync);
        ImGui::Text("FPS: %.1f", fps);

        // Graph (Mini FPS history)
        ImGui::PlotLines("##FPSGraph", fpsHistory.data(), historySize, historyOffset, nullptr, 0.0f, 120.0f, ImVec2(300, 80));

        ImGui::Text("mouse : %4d, %4d", mouseX, mouseY);
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}