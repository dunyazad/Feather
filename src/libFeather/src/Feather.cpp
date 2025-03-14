#include <Feather.h>

#include <Entity.h>
#include <MiniMath.h>
#include <Monitor.h>
#include <FeatherWindow.h>
#include <Component/Components.h>
#include <Component/Shader.h>
#include <System/Systems.h>

unordered_map<type_index, vector<FeatherObject*>> libFeather::s_instanceMap;

libFeather::libFeather() {}
libFeather::~libFeather() {}

void libFeather::Initialize(ui32 width, ui32 height)
{
    featherWindow = new FeatherWindow();
    featherWindow->Initialize(width, height);
    
    CreateInstance<EventSystem>("EventSystem", featherWindow)->Initialize();
    CreateInstance<GUISystem>("GUISystem", featherWindow)->Initialize();
    CreateInstance<InputSystem>("InputSystem", featherWindow)->Initialize();
    CreateInstance<ImmediateModeRenderSystem>("ImmediateModeRenderSystem", featherWindow)->Initialize();
    CreateInstance<RenderSystem>("RenderSystem", featherWindow)->Initialize();

    /////////glfwSwapInterval(0);  // Disable V-Sync
}

void libFeather::Terminate()
{
    for (auto& instance : objectInstances)
    {
        if (nullptr != instance)
        {
            delete instance;
        }
        objectInstances.clear();
    }

    if (nullptr != featherWindow)
    {
        delete featherWindow;
    }
}

void libFeather::Run()
{
#ifdef _WINDOWS
    MaximizeConsoleWindowOnMonitor(1);
    
    MaximizeWindowOnMonitor(glfwGetWin32Window(featherWindow->GetGLFWwindow()), 2);
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

        for (auto& system : GetInstances<SystemBase>())
        {
            system->Update(frameNo, timeDelta);
        }

        glfwSwapBuffers(glfwGetCurrentContext());

        frameNo++;
        lastTime = now;
    }
}

unordered_map<type_index, vector<FeatherObject*>>& libFeather::GetInstanceMap()
{
    return s_instanceMap;
}

set<FeatherObject*> libFeather::GetAllInstances()
{
    set<FeatherObject*> allInstances;
    for (const auto& [type, instances] : s_instanceMap)
    {
        allInstances.insert(instances.begin(), instances.end());
    }
    return allInstances;
}