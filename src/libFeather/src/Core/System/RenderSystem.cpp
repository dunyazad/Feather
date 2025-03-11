#include <Core/System/RenderSystem.h>

RenderSystem::RenderSystem(FeatherWindow* window)
	: SystemBase(window)
{
}

RenderSystem::~RenderSystem()
{
}

void RenderSystem::Initialize()
{
    CreateShader();
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
}

Shader* RenderSystem::CreateShader()
{
    auto s = new Shader();
    s->Initialize();
    shaders.push_back(s);
    return s;
}
