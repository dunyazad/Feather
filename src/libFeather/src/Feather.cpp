#include <Feather.h>

#include <Monitor.h>
#include <FeatherWindow.h>
#include <Buffer.h>
#include <Component/Components.h>
#include <Component/Shader.h>
#include <System/Systems.h>

libFeather::libFeather() {}
libFeather::~libFeather() {}

void libFeather::Initialize(ui32 width, ui32 height)
{
    featherWindow = new FeatherWindow();
    featherWindow->Initialize(width, height);

    frameBuffer = new FrameBuffer();
    frameBuffer->Initialize(width, height, true);

    inputSystem = new InputSystem(featherWindow);
    inputSystem->Initialize();
    eventSystem = new EventSystem(featherWindow);
    eventSystem->Initialize();
    renderSystem = new RenderSystem(featherWindow);
    renderSystem->Initialize();
    immediateModeRenderSystem = new ImmediateModeRenderSystem(featherWindow);
    immediateModeRenderSystem->Initialize();
    guiSystem = new GUISystem(featherWindow);
    guiSystem->Initialize();

    CreateShader("ScreenQuadShader",
        File("../../res/Shaders/ScreenQuad.vs"),
        File("../../res/Shaders/ScreenQuad.fs"));

    SetupScreenQuad();

    /////////glfwSwapInterval(0);  // Disable V-Sync
}

void libFeather::Terminate()
{
    for (auto& [name, shader] : shaders)
    {
        if (nullptr != shader) delete shader;
    }
    shaders.clear();

    if (nullptr != frameBuffer)
    {
        frameBuffer->Terminate();
        delete frameBuffer;
		frameBuffer = nullptr;
    }

    if (nullptr != inputSystem)
    {
        delete inputSystem;
        inputSystem = nullptr;
    }
    if (nullptr != eventSystem)
    {
        delete eventSystem;
		eventSystem = nullptr;
    }
    if (nullptr != renderSystem)
    {
        delete renderSystem;
		renderSystem = nullptr;
    }
    if (nullptr != immediateModeRenderSystem)
    {
        delete immediateModeRenderSystem;
		immediateModeRenderSystem = nullptr;
    }
    if (nullptr != guiSystem)
    {
        delete guiSystem;
        guiSystem = nullptr;
    }

    if (nullptr != featherWindow)
    {
        delete featherWindow;
        featherWindow = nullptr;
    }
}

void libFeather::Run()
{
#ifdef _WINDOWS
    MaximizeConsoleWindowOnMonitor(consoleWindowIndex);
    
    MaximizeWindowOnMonitor(glfwGetWin32Window(featherWindow->GetGLFWwindow()), mainWindowIndex);
#endif

    for (auto& callback : onInitializeCallbacks)
    {
        callback();
    }

    ui32 frameNo = 0;
    auto lastTime = Time::Now();

    glClearColor(XYZW(clearColor));

    Shader* screenShader = GetShader("ScreenQuadShader");
    if (screenShader == nullptr)
    {
        printf("CRITICAL ERROR: 'screenQuadShader' not found!\n");
        return;
    }
    screenShader->Use();
    screenShader->UniformInt(screenShader->GetUniformLocation("screenTexture"), 0);

    while (!glfwWindowShouldClose(glfwGetCurrentContext()))
    {
        auto now = Time::Now();
        auto timeDelta = (f32)(Time::Microseconds(lastTime, now)) / 1000.0f;

        for (auto& callback : onUpdateCallbacks)
        {
            callback(timeDelta);
        }

        frameBuffer->Bind();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glfwPollEvents();

		inputSystem->Update(frameNo, timeDelta);
        eventSystem->Update(frameNo, timeDelta);
        renderSystem->Update(frameNo, timeDelta);
        immediateModeRenderSystem->Update(frameNo, timeDelta);
        guiSystem->Update(frameNo, timeDelta);

        frameBuffer->Unbind();

        int fbWidth = 0, fbHeight = 0;
        glfwGetFramebufferSize(featherWindow->GetGLFWwindow(), &fbWidth, &fbHeight);
        glViewport(0, 0, fbWidth, fbHeight);

        glDisable(GL_DEPTH_TEST);

        glClearColor(XYZW(clearColor));
        glClear(GL_COLOR_BUFFER_BIT);

        screenShader->Use();

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, frameBuffer->GetColorTex());

        DrawScreenQuad();

        glfwSwapBuffers(glfwGetCurrentContext());

        frameNo++;
        lastTime = now;
    }
}

void libFeather::SetupScreenQuad()
{
    float quadVertices[] = {
        // positions   // texcoords
        -1.0f,  1.0f,  0.0f, 1.0f,
        -1.0f, -1.0f,  0.0f, 0.0f,
         1.0f, -1.0f,  1.0f, 0.0f,
         1.0f,  1.0f,  1.0f, 1.0f
    };

    unsigned int indices[] = { 0, 1, 2, 0, 2, 3 };

    glGenVertexArrays(1, &quadVAO);
    glGenBuffers(1, &quadVBO);
    glGenBuffers(1, &quadEBO);

    glBindVertexArray(quadVAO);

    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, quadEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Position (layout = 0)
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // TexCoords (layout = 1)
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void*)(2 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void libFeather::DrawScreenQuad()
{
    glBindVertexArray(quadVAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, quadEBO); // 안전하게 명시
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
        printf("[DrawScreenQuad] GL ERROR: %d\n", err);
}

void libFeather::OnFrameBufferResize(int width, int height)
{
    frameBuffer->Resize(width, height);
    glViewport(0, 0, width, height);
}

Entity libFeather::CreateEntity(const std::string& name)
{
    auto it = nameEntityMapping.find(name);
    if (it == nameEntityMapping.end())
    {
        auto entity = registry.create();
        nameEntityMapping[name] = entity;
        entityNameMapping[entity] = name;
        return entity;
    }
    else
    {
        return (*it).second;
    }
}

Entity libFeather::GetEntityByName(const std::string& name)
{
    auto it = nameEntityMapping.find(name);
    if (it != nameEntityMapping.end())
    {
        return (*it).second;
    }
    else
    {
        return InvalidEntity;
    }
}

const std::string& libFeather::GetEntityName(Entity entity)
{
    auto it = entityNameMapping.find(entity);
    if (it != entityNameMapping.end())
    {
        return (*it).second;
    }
    else
    {
        return EmptyString;
    }
}

void libFeather::RemoveEntity(const std::string& name)
{
    auto it = nameEntityMapping.find(name);
    if (it != nameEntityMapping.end())
    {
        auto entity = (*it).second;
        registry.destroy(entity);
        nameEntityMapping.erase(it);
        entityNameMapping.erase(entity);
    }
}

void libFeather::RemoveEntity(Entity entity)
{
    auto it = entityNameMapping.find(entity);
    if (it != entityNameMapping.end())
    {
        auto name = (*it).second;
        registry.destroy((*it).first);
        nameEntityMapping.erase(name);
        entityNameMapping.erase(it);
    }
}

Shader* libFeather::CreateShader(const std::string& name, const File& vsFile, const File& gsFile, const File& fsFile)
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

Shader* libFeather::CreateShader(const std::string& name, const File& vsFile, const File& fsFile)
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

Shader* libFeather::GetShader(const std::string& name)
{
    if (0 != shaders.count(name)) return shaders[name];
    else return nullptr;
}
