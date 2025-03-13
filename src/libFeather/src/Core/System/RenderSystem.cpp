#include <Core/System/RenderSystem.h>
#include <Feather.h>
#include <Core/Component/Components.h>

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
    auto cameras = Feather::GetInstance().GetComponents<PerspectiveCamera>();
    for (auto& camera : cameras)
    {
        camera->Update(frameNo, timeDelta);
    }
}
