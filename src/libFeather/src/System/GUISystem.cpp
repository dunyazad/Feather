#include <System/GUISystem.h>
#include <FeatherWindow.h>
#include <Feather.h>
#include <Component/GUIComponent/GUIComponents.h>

float amplitude = 1.0f;
float frequency = 1.0f;

GUISystem::GUISystem(FeatherWindow* window)
	: RegisterDerivation<GUISystem, SystemBase>(window)
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

void GUISystem::Update(ui32 frameNo, f32 timeDelta)
{
    if (needFontReload) {
        ReloadFont(fontSize);
        needFontReload = false;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    auto components = Feather.GetComponents<StatusPanel>();
    for (auto& component : components)
    {
        component->Render();
    }

    ShowUIPanel();
    ShowGraphPanel();
    //ShowFPS();
    //ShowTeapotPanel();

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

void GUISystem::ShowFPS()
{
    static const int historySize = 50;  // Number of FPS values to store
    static std::vector<float> fpsHistory(historySize, 60.0f);
    static int historyOffset = 0;
    static float accumulatedFPS = 0.0f;
    static int frameCount = 0;
    static const int updateRate = 10;

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
        float memoryUsage = GetMemoryUsageMB();
        ImGui::Text("Mem: %.1fmb", memoryUsage);

        // FPS Display with White Text
        ImGui::Text("FPS: %.1f", fps);

        // Graph (Mini FPS history)
        ImGui::PlotLines("##FPSGraph", fpsHistory.data(), historySize, historyOffset, nullptr, 0.0f, 120.0f, ImVec2(100, 80));

        // Add a settings or alert icon (Dummy Icon)
        ImGui::SameLine();
        ImGui::Text("O");
    }
    ImGui::End();

    ImGui::PopStyleColor();
    ImGui::PopStyleVar();
}