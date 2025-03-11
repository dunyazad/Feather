#include <libFeather.h>

#include <Core/Entity.h>
#include <Core/MiniMath.h>
#include <Core/FeatherWindow.h>
#include <Core/Shader.h>
#include <Core/System/SystemBase.h>
#include <Core/System/GUISystem.h>
#include <Core/System/RenderSystem.h>

libFeather* Feather::s_instance = nullptr;

#ifdef _WINDOWS
BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData) {
    std::vector<MonitorInfo>* monitors = reinterpret_cast<std::vector<MonitorInfo>*>(dwData);
    MONITORINFO monitorInfo;
    monitorInfo.cbSize = sizeof(MONITORINFO);
    if (GetMonitorInfo(hMonitor, &monitorInfo)) {
        monitors->push_back({ hMonitor, monitorInfo });
    }
    return TRUE;
}

void MaximizeConsoleWindowOnMonitor(int monitorIndex) {
    HWND consoleWindow = GetConsoleWindow();
    if (!consoleWindow) return;

    std::vector<MonitorInfo> monitors;
    EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, reinterpret_cast<LPARAM>(&monitors));

    if (monitorIndex >= 0 && monitorIndex < monitors.size()) {
        const MonitorInfo& monitor = monitors[monitorIndex];
        RECT workArea = monitor.monitorInfo.rcWork;

        MoveWindow(consoleWindow, workArea.left, workArea.top,
            workArea.right - workArea.left,
            workArea.bottom - workArea.top, TRUE);
        ShowWindow(consoleWindow, SW_MAXIMIZE);
    }
}

void MaximizeWindowOnMonitor(HWND hwnd, int monitorIndex) {
    std::vector<MonitorInfo> monitors;
    EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, reinterpret_cast<LPARAM>(&monitors));

    if (monitorIndex >= 0 && monitorIndex < monitors.size()) {
        const MonitorInfo& monitor = monitors[monitorIndex];
        RECT workArea = monitor.monitorInfo.rcWork;

        // Set the position and size of the VTK window to match the monitor's work area
        //renderWindow->SetPosition(workArea.left, workArea.top);
        //renderWindow->SetSize(workArea.right - workArea.left, workArea.bottom - workArea.top);

        MoveWindow(hwnd, workArea.left, workArea.top,
            workArea.right - workArea.left,
            workArea.bottom - workArea.top, TRUE);

        ShowWindow(hwnd, SW_MAXIMIZE);
    }
}
#endif

libFeather::libFeather()
{
    Feather::s_instance = this;
}

libFeather::~libFeather()
{
}

void libFeather::Initialize(ui32 width, ui32 height)
{
    featherWindow = new FeatherWindow();
    featherWindow->Initialize(width, height);
    
    systems["RenderSystem"] = new RenderSystem(featherWindow);
    systems["GUISystem"] = new GUISystem(featherWindow);

    for (auto& kvp : systems)
    {
        kvp.second->Initialize();
    }

    /////////glfwSwapInterval(0);  // Disable V-Sync
}

void libFeather::Terminate()
{
    for (auto& kvp : systems)
    {
        if (nullptr != kvp.second)
        {
            kvp.second->Terminate();
            delete kvp.second;
        }
    }
    systems.clear();

    if (nullptr != featherWindow)
    {
        delete featherWindow;
    }
}

Entity* libFeather::CreateEntity()
{
    auto entity = new Entity(numberOfEntities++);
    entities[entity->GetID()] = entity;
    return entity;
}

Entity* libFeather::GetEntity(EntityID id)
{
    if (0 == entities.count(id))
    {
        return nullptr;
    }
    else
    {
        return entities[id];
    }
}

void libFeather::Run()
{
#ifdef _WINDOWS
    MaximizeConsoleWindowOnMonitor(1);
    
    MaximizeWindowOnMonitor(glfwGetWin32Window(featherWindow->GetGLFWwindow()), 3);
#endif

    ui32 frameNo = 0;
    auto lastTime = Time::Now();

    glClearColor(0.3f, 0.6f, 0.9f, 1.0f);

    while (!glfwWindowShouldClose(glfwGetCurrentContext()))
    {
        auto now = Time::Now();
        auto timeDelta = (f32)(Time::Microseconds(lastTime, now)) / 1000.0f;

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glfwPollEvents();

        for (auto& kvp : systems)
        {
            kvp.second->Update(frameNo, timeDelta);
        }

        glfwSwapBuffers(glfwGetCurrentContext());

        frameNo++;
        lastTime = now;
    }
}
