#include <Feather.h>

#include <MiniMath.h>
#include <Monitor.h>
#include <FeatherWindow.h>
#include <Component/Components.h>
#include <Component/Shader.h>
#include <System/Systems.h>

libFeather::libFeather() {}
libFeather::~libFeather() {}

GUISystem* guiSystem;

void libFeather::Initialize(ui32 width, ui32 height)
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return;
    }

    featherWindow = new FeatherWindow();
    featherWindow->Initialize(width, height);

    EventDispatcher::SetFeatherWindow(featherWindow);
    EventDispatcher::RegisterGLFWCallbacks();

    guiSystem = new GUISystem(featherWindow);
    guiSystem->Initialize();

    /////////glfwSwapInterval(0);  // Disable V-Sync
}

void libFeather::Terminate()
{
    if (nullptr != guiSystem)
    {
        guiSystem->Terminate();
        delete guiSystem;
        guiSystem = nullptr;
    }

    if (nullptr != featherWindow)
    {
        featherWindow->Terminate();
        delete featherWindow;
        featherWindow = nullptr;
    }

    glfwTerminate();
}

ui32 frameNo = 0;
auto lastTime = Time::Now();

void idle_cb(uv_idle_t* handle)
{
    if (glfwWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow()))
    {
        uv_stop(Feather.GetLoop());
    }
    
    glfwPollEvents();

    auto now = Time::Now();
    auto timeDelta = (f32)(Time::Microseconds(lastTime, now)) / 1000.0f;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    guiSystem->Update(frameNo, timeDelta);

    //auto eventSystem = Feather.GetFirstInstance<EventSystem>();
    //if (nullptr != eventSystem) eventSystem->Update(frameNo, timeDelta);

    //auto inputSystem = Feather.GetFirstInstance<InputSystem>();
    //if (nullptr != inputSystem) inputSystem->Update(frameNo, timeDelta);

    //auto renderSystem = Feather.GetFirstInstance<RenderSystem>();
    //if (nullptr != renderSystem) renderSystem->Update(frameNo, timeDelta);

    //auto immediateModeRenderSystem = Feather.GetFirstInstance<ImmediateModeRenderSystem>();
    //if (nullptr != immediateModeRenderSystem) immediateModeRenderSystem->Update(frameNo, timeDelta);

    //auto guiSystem = Feather.GetFirstInstance<GUISystem>();
    //if (nullptr != guiSystem) guiSystem->Update(frameNo, timeDelta);

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

    loop = uv_default_loop();

    uv_idle_init(loop, &idle_handle);
    uv_idle_start(&idle_handle, idle_cb);

    /*glfwSetCursorPosCallback(featherWindow->GetGLFWwindow(),
        bind(&libFeather::mouseCallback, this,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));*/

    glClearColor(0.3f, 0.6f, 0.9f, 1.0f);

    uv_run(loop, UV_RUN_DEFAULT);

    uv_idle_stop(&idle_handle);
}
