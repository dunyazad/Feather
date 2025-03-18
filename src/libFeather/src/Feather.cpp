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
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return;
    }

    featherWindow = new FeatherWindow();
    featherWindow->Initialize(width, height);
    
    CreateInstance<EventSystem>("EventSystem", featherWindow)->Initialize();
    CreateInstance<InputSystem>("InputSystem", featherWindow)->Initialize();
    CreateInstance<RenderSystem>("RenderSystem", featherWindow)->Initialize();
    CreateInstance<ImmediateModeRenderSystem>("ImmediateModeRenderSystem", featherWindow)->Initialize();
    CreateInstance<GUISystem>("GUISystem", featherWindow)->Initialize();

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
    }
    objectInstances.clear();

    for (auto& pair : s_instanceMap)
    {
        pair.second.clear();
    }
    s_instanceMap.clear();

    if (nullptr != featherWindow)
    {
        featherWindow->Terminate();

        delete featherWindow;
    }

    glfwTerminate();
}

ui32 frameNo = 0;
auto lastTime = Time::Now();

void glfw_poll_cb(uv_poll_t* handle, int status, int events) {
    if (glfwWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow())) {
        uv_stop(Feather.GetLoop());
    }

	auto now = Time::Now();
	auto timeDelta = (f32)(Time::Microseconds(lastTime, now)) / 1000.0f;

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glfwPollEvents();

	auto eventSystem = Feather.GetFirstInstance<EventSystem>();
	if (nullptr != eventSystem) eventSystem->Update(frameNo, timeDelta);

	auto inputSystem = Feather.GetFirstInstance<InputSystem>();
	if (nullptr != inputSystem) inputSystem->Update(frameNo, timeDelta);

	auto renderSystem = Feather.GetFirstInstance<RenderSystem>();
	if (nullptr != renderSystem) renderSystem->Update(frameNo, timeDelta);

	auto immediateModeRenderSystem = Feather.GetFirstInstance<ImmediateModeRenderSystem>();
	if (nullptr != immediateModeRenderSystem) immediateModeRenderSystem->Update(frameNo, timeDelta);

	auto guiSystem = Feather.GetFirstInstance<GUISystem>();
	if (nullptr != guiSystem) guiSystem->Update(frameNo, timeDelta);

	glfwSwapBuffers(glfwGetCurrentContext());

	frameNo++;
	lastTime = now;

    glfwPollEvents();
}

void idle_cb(uv_idle_t* handle) {
    if (glfwWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow())) {
        uv_stop(Feather.GetLoop());
    }
    
    glfwPollEvents();

    auto now = Time::Now();
    auto timeDelta = (f32)(Time::Microseconds(lastTime, now)) / 1000.0f;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    auto eventSystem = Feather.GetFirstInstance<EventSystem>();
    if (nullptr != eventSystem) eventSystem->Update(frameNo, timeDelta);

    auto inputSystem = Feather.GetFirstInstance<InputSystem>();
    if (nullptr != inputSystem) inputSystem->Update(frameNo, timeDelta);

    auto renderSystem = Feather.GetFirstInstance<RenderSystem>();
    if (nullptr != renderSystem) renderSystem->Update(frameNo, timeDelta);

    auto immediateModeRenderSystem = Feather.GetFirstInstance<ImmediateModeRenderSystem>();
    if (nullptr != immediateModeRenderSystem) immediateModeRenderSystem->Update(frameNo, timeDelta);

    auto guiSystem = Feather.GetFirstInstance<GUISystem>();
    if (nullptr != guiSystem) guiSystem->Update(frameNo, timeDelta);

    glfwSwapBuffers(glfwGetCurrentContext());

    frameNo++;
    lastTime = now;
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

    loop = uv_default_loop();

    uv_idle_init(loop, &idle_handle);
    uv_idle_start(&idle_handle, idle_cb);

    glClearColor(0.3f, 0.6f, 0.9f, 1.0f);

    uv_run(loop, UV_RUN_DEFAULT);

    uv_idle_stop(&idle_handle);
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