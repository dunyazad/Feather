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
    const std::map<Shader*, std::vector<T*>>& shadermap)
{
    for (auto& [shader, renderables] : shadermap)
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
    const std::map<Shader*, std::vector<Renderable*>>& shadermap)
{
    RenderRenderablesTemplate<Renderable>(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shadermap);
}

void RenderSystem::RenderDebuggingRenderables(
    ui32 frameNo, f32 timeDelta,
    const glm::mat4& viewMatrix,
    const glm::mat4& perspectiveMatrix,
    const glm::vec3& eye,
    const std::map<Shader*, std::vector<DebuggingRenderable*>>& shadermap)
{
    RenderRenderablesTemplate<DebuggingRenderable>(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shadermap);
}

void RenderSystem::Update(ui32 frameNo, f32 timeDelta)
{
    glm::mat4 viewMatrix;
    glm::mat4 perspectiveMatrix;
    glm::vec3 eye;

    auto& registry = Feather.GetRegistry();

    {
        auto entites = registry.view<PerspectiveCamera>();
        for (auto& entity : entites)
        {
            auto& camera = entites.get<PerspectiveCamera>(entity);

            camera.Update(frameNo, timeDelta);

            viewMatrix = camera.GetViewMatrix();
            perspectiveMatrix = camera.GetProjectionMatrix();
            eye = camera.GetEye();
        }
    }

    {
        std::map<Shader*, std::vector<Renderable*>> shadermap;
        for (auto& entity : registry.view<Renderable>())
        {
            auto& r = registry.get<Renderable>(entity);
            if (!r.IsUsingAlpha())
                shadermap[r.GetActiveShader()].push_back(&r);
        }

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);      // Z-buffer write ON
        glDisable(GL_BLEND);       // Blend OFF

        RenderRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shadermap);
    }
    {
        struct ZRenderable { Renderable* r; float z; };
        std::vector<ZRenderable> transparentObjs;

        for (auto& entity : registry.view<Renderable>())
        {
            auto& r = registry.get<Renderable>(entity);
            if (r.IsUsingAlpha())
            {
                // 월드 좌표계 Z값(eye 기준 거리, 혹은 view-space z) 추출
                glm::vec3 pos = r.GetVertices().empty() ? glm::vec3(0) : r.GetVertices().at(0);
                float z = glm::length(eye - pos);
                transparentObjs.push_back({ &r, z });
            }
        }

        // 멀리→가까이 정렬
        std::sort(transparentObjs.begin(), transparentObjs.end(), [](const ZRenderable& a, const ZRenderable& b)
            {
                return a.z > b.z;
            });

        std::map<Shader*, std::vector<Renderable*>> shadermap;
        for (auto& t : transparentObjs)
        {
            shadermap[t.r->GetActiveShader()].push_back(t.r);
        }

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);     // Z-buffer write OFF
        glEnable(GL_BLEND);        // Blend ON
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        RenderRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shadermap);

        glDepthMask(GL_TRUE);
    }

    {
        std::map<Shader*, std::vector<DebuggingRenderable*>> shadermap;
        for (auto& entity : registry.view<DebuggingRenderable>())
        {
            auto& r = registry.get<DebuggingRenderable>(entity);
            if (!r.IsUsingAlpha())
                shadermap[r.GetActiveShader()].push_back(&r);
        }

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);      // Z-buffer write ON
        glDisable(GL_BLEND);       // Blend OFF

        RenderDebuggingRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shadermap);
    }
    {
        struct ZRenderable { DebuggingRenderable* r; float z; };
        std::vector<ZRenderable> transparentObjs;

        for (auto& entity : registry.view<DebuggingRenderable>())
        {
            auto& r = registry.get<DebuggingRenderable>(entity);
            if (r.IsUsingAlpha())
            {
                // 월드 좌표계 Z값(eye 기준 거리, 혹은 view-space z) 추출
                glm::vec3 pos = r.GetVertices().empty() ? glm::vec3(0) : r.GetVertices().at(0);
                float z = glm::length(eye - pos);
                transparentObjs.push_back({ &r, z });
            }
        }

        // 멀리→가까이 정렬
        std::sort(transparentObjs.begin(), transparentObjs.end(), [](const ZRenderable& a, const ZRenderable& b)
            {
                return a.z > b.z;
            });

        std::map<Shader*, std::vector<DebuggingRenderable*>> shadermap;
        for (auto& t : transparentObjs)
        {
            shadermap[t.r->GetActiveShader()].push_back(t.r);
        }

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_FALSE);     // Z-buffer write OFF
        glEnable(GL_BLEND);        // Blend ON
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        RenderDebuggingRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, shadermap);

        glDepthMask(GL_TRUE);
    }
}