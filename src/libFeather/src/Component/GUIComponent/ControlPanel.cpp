#include<Component/GUIComponent/ControlPanel.h>

ControlPanel::ControlPanel(const std::string& title) : title(title)
{
}

ControlPanel::~ControlPanel()
{
}

void ControlPanel::Render()
{
    //ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.7f); // Semi-transparent background

    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 12.0f); // Rounded corners
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.3f, 0.3f, 0.6f, 0.8f)); // Green background

    if (ImGui::Begin("Performance Overlay", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove))
    {
        for (auto& button : buttons)
        {
            if (ImGui::Button(button.label.c_str(), { button.width, button.height }))
            {
                for (auto& callback : button.callbacks)
                {
                    callback();
                }
			}
        }
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}

void ControlPanel::AddButton(const std::string& label, float width, float height, const std::function<void()>& callback)
{
    buttons.push_back({ label, width, height, { callback } });
}