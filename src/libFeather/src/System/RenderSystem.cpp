#include <System/RenderSystem.h>
#include <Feather.h>
#include <Component/Components.h>

RenderSystem::RenderSystem(FeatherWindow* window)
    : RegisterDerivation<RenderSystem, SystemBase>(window)
{
}

RenderSystem::~RenderSystem()
{
}

void RenderSystem::Initialize()
{
}

void RenderSystem::Terminate()
{
    for (auto& s : shaders)
    {
        if (nullptr != s)
        {
            delete s;
        }
    }
    shaders.clear();
}

void RenderSystem::Update(ui32 frameNo, f32 timeDelta)
{
    auto cameras = Feather.GetInstances<PerspectiveCamera>();
    if (cameras.empty())
    {
        alog("No camera !!!\n");
        return;
    }

    for (auto& camera : cameras)
    {
        camera->Update(frameNo, timeDelta);
    }

    map<Shader*, vector<Renderable*>> shaderMapping;
    auto renderables = Feather.GetInstances<Renderable>();
    for (auto& renderable : renderables)
    {
        auto shader = renderable->GetActiveShader();
        shaderMapping[shader].push_back(renderable);
    }

    //glEnable(GL_DEPTH_TEST);
    //glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glPointSize(5.0f);
    glLineWidth(2.0f);

    glDisable(GL_BLEND);

    for (auto& [shader, renderables] : shaderMapping)
    {
        if (nullptr == shader) continue;

        shader->Use();
        {
            auto index = shader->GetUniformLocation("model");
            shader->UniformM4(index, MiniMath::M4::identity());
        }
        {
            auto index = shader->GetUniformLocation("view");
            shader->UniformM4(index, (*cameras.begin())->GetViewMatrix());
        }
        {
            auto index = shader->GetUniformLocation("projection");
            shader->UniformM4(index, (*cameras.begin())->GetProjectionMatrix());
        }
        {
            auto index = shader->GetUniformLocation("cameraPos");
            shader->UniformV3(index, (*cameras.begin())->GetEye());
        }

        for (auto& renderable : renderables)
        {
            renderable->Update(frameNo, timeDelta);
            renderable->Draw();
        }
    }

    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
    {
        std::cerr << "OpenGL Error: " << err << std::endl;
    }
}
