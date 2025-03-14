#include <System/RenderSystem.h>
#include <Feather.h>
#include <Component/Components.h>

RenderSystem::RenderSystem(FeatherWindow* window)
	: SystemBase(window)
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
    auto cameras = Feather.GetComponents<PerspectiveCamera>();
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
    auto renderables = Feather.GetComponents<Renderable>();
    for (auto& renderable : renderables)
    {
        auto shader = renderable->GetShader();
        shaderMapping[shader].push_back(renderable);
    }

    for (auto& kvp : shaderMapping)
    {
        if (nullptr == kvp.first) continue;

        kvp.first->Use();
        kvp.first->UniformM4(0, cameras.front()->GetProjectionMatrix());
        kvp.first->UniformM4(1, cameras.front()->GetViewMatrix());

        for (auto& renderable : kvp.second)
        {
            renderable->Update(frameNo, timeDelta);
            renderable->Draw();
        }
    }
}
