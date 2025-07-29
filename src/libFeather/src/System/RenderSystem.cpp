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

template<typename T>
void RenderRenderablesTemplate(
    ui32 frameNo, f32 timeDelta,
    const glm::mat4& viewMatrix,
    const glm::mat4& perspectiveMatrix,
    const glm::vec3& eye,
    const map<Shader*, vector<T*>>& shaderMapping)
{
    for (auto& [shader, renderables] : shaderMapping)
    {
        if (nullptr == shader) continue;

        shader->Use();

        for (auto& renderable : renderables)
        {
            renderable->Update(frameNo, timeDelta);

            auto entity = Feather.GetEntityByComponent<T>(renderable);
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
                    shader->UniformM4(index, glm::identity<glm::mat4>());
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

            renderable->Draw(shader);

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

void RenderSystem::RenderRenderables(
    ui32 frameNo, f32 timeDelta,
    const glm::mat4& viewMatrix,
    const glm::mat4& perspectiveMatrix,
    const glm::vec3& eye,
    const map<Shader*, vector<Renderable*>>& shaderMapping)
{
    RenderRenderablesTemplate<Renderable>(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shaderMapping);
}

void RenderSystem::RenderDebuggingRenderables(
    ui32 frameNo, f32 timeDelta,
    const glm::mat4& viewMatrix,
    const glm::mat4& perspectiveMatrix,
    const glm::vec3& eye,
    const map<Shader*, vector<DebuggingRenderable*>>& shaderMapping)
{
    RenderRenderablesTemplate<DebuggingRenderable>(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shaderMapping);
}

void RenderSystem::Update(ui32 frameNo, f32 timeDelta)
{
    glm::mat4 viewMatrix;
    glm::mat4 perspectiveMatrix;
    glm::vec3 eye;

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

    {
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

        RenderRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shaderMapping);
    }

    {
        map<Shader*, vector<DebuggingRenderable*>> shaderMapping;
        auto entities = Feather.GetRegistry().view<DebuggingRenderable>();
        for (auto& entity : entities)
        {
            auto& renderable = Feather.GetRegistry().get<DebuggingRenderable>(entity);
            auto shader = renderable.GetActiveShader();
            shaderMapping[shader].push_back(&renderable);
        }

        //glEnable(GL_DEPTH_TEST);
        //glEnable(GL_BLEND);
        //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glPointSize(5.0f);
        glLineWidth(3.0f);

        glDisable(GL_BLEND);

        RenderDebuggingRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shaderMapping);
    }
}
