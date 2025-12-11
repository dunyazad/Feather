#include<Component/GUIComponent/TreeViewPanel.h>
#include <Feather.h>
#include <FeatherWindow.h>
#include <Component/Camera.h>

TreeViewPanel::TreeViewPanel()
{
}

TreeViewPanel::~TreeViewPanel()
{
}

void TreeViewPanel::Render()
{
    bool enabledRoot = true;
    bool enabledChild1 = false;
    bool enabledChild2 = true;

    if (ImGui::TreeNode("Root Node"))
    {
        ImGui::Checkbox("Enable Root", &enabledRoot);

        if (ImGui::TreeNode("Child 1"))
        {
            ImGui::Checkbox("Enable Child 1", &enabledChild1);
            ImGui::TreePop();
        }

        if (ImGui::TreeNode("Child 2"))
        {
            ImGui::Checkbox("Enable Child 2", &enabledChild2);
            ImGui::TreePop();
        }

        ImGui::TreePop();
    }
}
