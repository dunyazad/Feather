#include <Feather.h>

#include <Core/Entity.h>
#include <Core/MiniMath.h>
#include <Core/FeatherWindow.h>
#include <Core/Shader.h>
#include <Core/Component/Components.h>
#include <Core/System/Systems.h>

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

Feather::Feather() {}
Feather::~Feather() {}

void Feather::Initialize(ui32 width, ui32 height)
{
    featherWindow = new FeatherWindow();
    featherWindow->Initialize(width, height);
    
    systems[typeid(GUISystem)] = new GUISystem(featherWindow);
    systems[typeid(EventSystem)] = new EventSystem(featherWindow);
    systems[typeid(InputSystem)] = new InputSystem(featherWindow);
    systems[typeid(ImmediateModeRenderSystem)] = new ImmediateModeRenderSystem(featherWindow);
    systems[typeid(RenderSystem)] = new RenderSystem(featherWindow);

    for (auto& kvp : systems)
    {
        kvp.second->Initialize();
    }

    /////////glfwSwapInterval(0);  // Disable V-Sync
}

void Feather::Terminate()
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

Entity* Feather::CreateEntity(const string& name)
{
    auto entity = new Entity(nextEntityID++, name);
    entities[entity->GetID()] = entity;
    return entity;
}

Entity* Feather::GetEntity(EntityID id)
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

ComponentBase* Feather::GetComponent(ComponentID id)
{
    if (0 == idComponentMapping.count(id))
    {
        return nullptr;
    }
    else
    {
        return components[idComponentMapping[id]];
    }
}

const vector<ui32>& Feather::GetComponentIDsByTypeIndex(const type_index& typeIndex)
{
    if (0 == typeComponentMapping.count(typeIndex))
    {
        return typeComponentMapping[typeid(ComponentBase)];
    }
    else
    {
        return typeComponentMapping[typeIndex];
    }
}

void Feather::Run()
{
#ifdef _WINDOWS
    MaximizeConsoleWindowOnMonitor(1);
    
    MaximizeWindowOnMonitor(glfwGetWin32Window(featherWindow->GetGLFWwindow()), 3);
#endif

    for (auto& callback : onInitializeCallbacks)
    {
        callback();
    }

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
