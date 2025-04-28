#include <System/RenderSystem.h>
#include <Feather.h>
#include <Component/Components.h>

RenderSystem::RenderSystem(FeatherWindow* window)
    : window(window)
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
}

void RenderSystem::Update(ui32 frameNo, f32 timeDelta)
{
    MiniMath::M4 viewMatrix;
    MiniMath::M4 perspectiveMatrix;
    MiniMath::V3 eye;

    auto& registry = Feather.GetRegistry();
    auto entites = registry.view<PerspectiveCamera>();
    for (auto& entity : entites)
    {
        auto& camera = entites.get<PerspectiveCamera>(entity);

        camera.Update(frameNo, timeDelta);

        viewMatrix = camera.GetViewMatrix();
        perspectiveMatrix = camera.GetProjectionMatrix();
        eye = camera.GetEye();
    }

    map<Shader*, vector<Renderable*>> shaderMapping;
    auto entities = Feather.GetRegistry().view<Renderable>();
    for (auto& entity : entities)
    {
        auto& renderable = Feather.GetRegistry().get<Renderable>(entity);
        auto shader = renderable.GetActiveShader();
        shaderMapping[shader].push_back(&renderable);
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
        
        for (auto& renderable : renderables)
        {
            renderable->Update(frameNo, timeDelta);

            auto entity = Feather.GetEntityByComponent<Renderable>(renderable);
            if (InvalidEntity == entity) continue;

            auto transform = Feather.GetComponent<Transform>(entity);
            if (nullptr != transform)
            {
                auto& transformMatrix = transform->GetTransformMatrix();

                auto index = shader->GetUniformLocation("model");
                if (-1 != index)
                {
                    shader->UniformM4(index, transformMatrix);
                }
            }
            else
            {
                auto index = shader->GetUniformLocation("model");
                if (-1 != index)
                {
                    shader->UniformM4(index, MiniMath::M4::identity());
                }
            }

            {
                auto index = shader->GetUniformLocation("view");
                if (-1 != index)
                {
                    shader->UniformM4(index, viewMatrix);
                }
            }
            {
                auto index = shader->GetUniformLocation("projection");
                if (-1 != index)
                {
                    shader->UniformM4(index, perspectiveMatrix);
                }
            }
            {
                auto index = shader->GetUniformLocation("cameraPos");
                if (-1 != index)
                {
                    shader->UniformV3(index, eye);
                }
            }

            auto texture = Feather.GetComponent<Texture>(entity);
            if (nullptr != texture)
            {
                texture->Bind();
             
                auto textureLocation = shader->GetUniformLocation("texture0");
                if (textureLocation != -1)
                {
                    glUniform1i(textureLocation, 0);
                }
            }

            renderable->Draw();

            if (nullptr != texture)
            {
                texture->Unbind();
            }
        }
    }

    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR)
    {
        std::cerr << "OpenGL Error: " << err << std::endl;
    }
}
