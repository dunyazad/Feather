#include<Component/GUIComponent/TextBlock.h>
#include <Feather.h>
#include <FeatherWindow.h>
#include <Component/Camera.h>

TextBlock::TextBlock()
{
}

TextBlock::~TextBlock()
{
}

void TextBlock::Render()
{
    Entity entity = Feather.GetEntityByName("Camera");
    auto camera = Feather.GetComponent<PerspectiveCamera>(entity);

    const glm::mat4& proj = camera->GetProjectionMatrix();
    const glm::mat4& view = camera->GetViewMatrix();

    ImFont* font = ImGui::GetFont();
    ImDrawList* draw_list = ImGui::GetForegroundDrawList();

    for (auto& info : textInfos)
    {
        glm::vec4 pos4(info.position, 1.0f);
        glm::vec4 clip = proj * view * pos4;

        if (clip.w == 0.0f)
            continue; // Avoid division by zero

        glm::vec3 ndc = glm::vec3(clip) / clip.w;
        if (clip.z < 0.0f)
            continue; // Behind camera

        if (ndc.x < -1.0f || ndc.x > 1.0f ||
            ndc.y < -1.0f || ndc.y > 1.0f ||
            ndc.z < 0.0f || ndc.z > 1.0f)
            continue; // NDC 범위 밖(화면 밖)

        float screenX = (ndc.x * 0.5f + 0.5f) * Feather.GetFeatherWindow()->GetWidth();
        float screenY = (1.0f - (ndc.y * 0.5f + 0.5f)) * Feather.GetFeatherWindow()->GetHeight();

        ImU32 col = IM_COL32(
            glm::clamp(info.color.x, 0.0f, 1.0f) * 255.0f,
            glm::clamp(info.color.y, 0.0f, 1.0f) * 255.0f,
            glm::clamp(info.color.z, 0.0f, 1.0f) * 255.0f,
            glm::clamp(info.color.w, 0.0f, 1.0f) * 255.0f
        );
        draw_list->AddText(font, info.fontSize, ImVec2(screenX, screenY), col, info.text.c_str());
    }
}

void TextBlock::AddText(const std::string& text, const glm::vec3& position, const glm::vec4& color, float fontSize)
{
    textInfos.push_back({ text, position, color, fontSize });
}

void TextBlock::Clear()
{
    textInfos.clear();
}