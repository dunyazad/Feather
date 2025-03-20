#include <System/GUISystem.h>
#include <FeatherWindow.h>
#include <Feather.h>
#include <Component/GUIComponent/GUIComponents.h>

float amplitude = 1.0f;
float frequency = 1.0f;

GUISystem::GUISystem(FeatherWindow* window)
    :window(window)
{
}

GUISystem::~GUISystem()
{
}

void GUISystem::Initialize()
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplGlfw_InitForOpenGL(window->GetGLFWwindow(), true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

void GUISystem::Terminate()
{
    ImPlot::DestroyContext();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
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

void GUISystem::Update(ui32 frameNo, f32 timeDelta)
{
    if (needFontReload) {
        ReloadFont(fontSize);
        needFontReload = false;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    Feather.GetRegistry().view<StatusPanel>().each([&](StatusPanel& panel) {
        ShowStatusPanel(panel);
        });

    ShowUIPanel();
    ShowGraphPanel();

    ImGui::Render();
    //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void GUISystem::ReloadFont(float newFontSize)
{
    ImGuiIO& io = ImGui::GetIO();
    io.Fonts->Clear();
    io.Fonts->AddFontFromFileTTF("../../res/Fonts/NanumGothic/NanumGothic-Regular.ttf", newFontSize);
    ImGui_ImplOpenGL3_DestroyFontsTexture();
    ImGui_ImplOpenGL3_CreateFontsTexture();
}

void GUISystem::ShowGraphPanel()
{
    ImGui::Begin("Sine Wave Plot");

    std::vector<float> x_data(100);
    std::vector<float> y_data(100);
    for (size_t i = 0; i < x_data.size(); i++) {
        x_data[i] = i * 0.1f;
        y_data[i] = amplitude * sin(frequency * x_data[i]);
    }

    if (ImPlot::BeginPlot("Sine Wave")) {
        ImPlot::PlotLine("sin(x)", x_data.data(), y_data.data(), x_data.size());
        ImPlot::EndPlot();
    }

    ImGui::End();
}

void GUISystem::ShowUIPanel()
{
    ImGui::Begin("Control Panel");

    if (ImGui::SliderFloat("Font Size", &fontSize, 12.0f, 32.0f)) {
        needFontReload = true;
    }

    if (ImGui::Button("Reset")) {
        amplitude = 1.0f;
        frequency = 1.0f;
        fontSize = 20.0f;
        needFontReload = true;
    }
    ImGui::End();
}

// Example memory usage function (replace with actual memory tracking if available)
float GetMemoryUsageMB() {
    return 41.0f;  // Dummy value, replace with real tracking
}

void GUISystem::ShowStatusPanel(StatusPanel& panel)
{
    float fps = ImGui::GetIO().Framerate;
    panel.accumulatedFPS += fps;
    panel.frameCount++;

    // Update the graph every few frames
    if (panel.frameCount >= panel.updateRate)
    {
        panel.fpsHistory[panel.historyOffset] = panel.accumulatedFPS / panel.updateRate;
        panel.historyOffset = (panel.historyOffset + 1) % panel.historySize;
        panel.accumulatedFPS = 0.0f;
        panel.frameCount = 0;
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
        ToggleButton("V-Sync", &panel.vSync);
        ImGui::Text("FPS: %.1f", fps);

        // Graph (Mini FPS history)
        ImGui::PlotLines("##FPSGraph", panel.fpsHistory.data(), panel.historySize, panel.historyOffset, nullptr, 0.0f, 120.0f, ImVec2(300, 80));

        ImGui::Text("mouse : %4d, %4d", panel.mouseX, panel.mouseY);
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}