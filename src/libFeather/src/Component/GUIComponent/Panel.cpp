#include<Component/GUIComponent/Panel.h>
#include <Feather.h>

Panel::Panel(const std::string& title)
    : title(title)
{
}

Panel::~Panel()
{
}

void Panel::Render()
{
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.7f); // Semi-transparent background

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 12.0f); // Rounded corners
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.3f, 0.3f, 0.6f, 0.8f)); // Green background

    if (ImGui::Begin(title.c_str(), nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove))
    {
        ImGui::Text("mouse : %4d, %4d", mouseX, mouseY);
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}
