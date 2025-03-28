#include <Feather.h>

#include <Entity.h>
#include <MiniMath.h>
#include <Monitor.h>
#include <FeatherWindow.h>
#include <Component/Components.h>
#include <Component/Shader.h>
#include <System/Systems.h>

libFeather::libFeather() {}
libFeather::~libFeather() {}

void libFeather::Initialize(ui32 width, ui32 height)
{
    featherWindow = new FeatherWindow();
    featherWindow->Initialize(width, height);

    eventSystem = new EventSystem(featherWindow);
    eventSystem->Initialize();
    renderSystem = new RenderSystem(featherWindow);
    renderSystem->Initialize();
    immediateModeRenderSystem = new ImmediateModeRenderSystem(featherWindow);
    immediateModeRenderSystem->Initialize();
    guiSystem = new GUISystem(featherWindow);
    guiSystem->Initialize();

    /////////glfwSwapInterval(0);  // Disable V-Sync
}

void libFeather::Terminate()
{
    for (auto& [name, shader] : shaders)
    {
        if (nullptr != shader) delete shader;
    }
    shaders.clear();

    if (nullptr != eventSystem) delete eventSystem;
    if (nullptr != renderSystem) delete renderSystem;
    if (nullptr != immediateModeRenderSystem) delete immediateModeRenderSystem;
    if (nullptr != guiSystem) delete guiSystem;

    if (nullptr != featherWindow) delete featherWindow;
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

        eventSystem->Update(frameNo, timeDelta);
        renderSystem->Update(frameNo, timeDelta);
        immediateModeRenderSystem->Update(frameNo, timeDelta);
        guiSystem->Update(frameNo, timeDelta);

        glfwSwapBuffers(glfwGetCurrentContext());

        frameNo++;
        lastTime = now;
    }
}

Shader* libFeather::CreateShader(const string& name, const File& vsFile, const File& gsFile, const File& fsFile)
{
    if (0 != shaders.count(name)) return shaders[name];
    else
    {
        auto shader = new Shader();
        shader->Initialize(vsFile, gsFile, fsFile);
        shaders[name] = shader;
        return shader;
    }
}

Shader* libFeather::CreateShader(const string& name, const File& vsFile, const File& fsFile)
{
    if (0 != shaders.count(name)) return shaders[name];
    else
    {
        auto shader = new Shader();
        shader->Initialize(vsFile, fsFile);
        shaders[name] = shader;
        return shader;
    }
}

Shader* libFeather::GetShader(const string& name)
{
    if (0 != shaders.count(name)) return shaders[name];
    else return nullptr;
}
