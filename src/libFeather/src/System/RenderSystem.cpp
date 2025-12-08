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

// [수정] 조명 정보(lightVector)를 인자로 추가
template<typename T>
void RenderRenderablesTemplate(
    ui32 frameNo, f32 timeDelta,
    const glm::mat4& viewMatrix,
    const glm::mat4& perspectiveMatrix,
    const glm::vec3& eye,
    const glm::vec4& lightVector,
    const std::map<Shader*, std::vector<T*>>& shadermap)
{
    for (auto& [shader, renderables] : shadermap)
    {
        if (nullptr == shader) continue;

        shader->Use();

        // [추가] 쉐이더에 조명 유니폼 설정
        // 쉐이더 코드에서 uniform vec4 lightPos; 라고 가정합니다.
        // w == 0.0이면 Directional, w == 1.0이면 Point Light로 처리
        auto lightLoc = shader->GetUniformLocation("lightPos");
        if (lightLoc != -1)
        {
            shader->UniformV4(lightLoc, lightVector);
        }

        for (auto& renderable : renderables)
        {
            renderable->Update(frameNo, timeDelta);

            auto entity = Feather.GetEntityByComponent<T>(renderable);
            if (InvalidEntity == entity) continue;

            auto transform = Feather.GetComponent<Transform>(entity);
            if (nullptr != transform)
            {
                auto& transformMatrix = transform->GetAbsoluteTransformMatrix();

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

// Wrapper 함수들도 lightVector를 받아서 넘겨주도록 수정
void RenderSystem::RenderRenderables(
    ui32 frameNo, f32 timeDelta,
    const glm::mat4& viewMatrix,
    const glm::mat4& perspectiveMatrix,
    const glm::vec3& eye,
    const glm::vec4& lightVector, // <--- 추가
    const std::map<Shader*, std::vector<Renderable*>>& shadermap)
{
    RenderRenderablesTemplate<Renderable>(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, lightVector, shadermap);
}

void RenderSystem::RenderDebuggingRenderables(
    ui32 frameNo, f32 timeDelta,
    const glm::mat4& viewMatrix,
    const glm::mat4& perspectiveMatrix,
    const glm::vec3& eye,
    const glm::vec4& lightVector, // <--- 추가
    const std::map<Shader*, std::vector<DebuggingRenderable*>>& shadermap)
{
    RenderRenderablesTemplate<DebuggingRenderable>(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, lightVector, shadermap);
}

void RenderSystem::Update(ui32 frameNo, f32 timeDelta)
{
    glm::mat4 viewMatrix;
    glm::mat4 perspectiveMatrix;
    glm::vec3 eye;

    // 현재 활성화된 카메라의 모드를 알기 위해 포인터 저장
    Camera* activeCamera = nullptr;

    auto& registry = Feather.GetRegistry();

    // 1. 카메라 업데이트 및 정보 수집
    {
        auto entites = registry.view<Camera>();
        for (auto& entity : entites)
        {
            auto& camera = entites.get<Camera>(entity);

            camera.Update(frameNo, timeDelta);

            viewMatrix = camera.GetViewMatrix();
            perspectiveMatrix = camera.GetProjectionMatrix();
            eye = camera.GetEye();

            activeCamera = &camera; // 마지막 업데이트된 카메라를 메인으로 간주
        }
    }

    // 2. 조명(Light) 정보 계산
    glm::vec4 lightUniformVector = glm::vec4(0.0f, 100.0f, 0.0f, 1.0f); // 기본값

    // 씬에 Light 컴포넌트가 있다면 가져오기 (없으면 위 기본값 사용)
    // 컴포넌트 이름이 Light라고 가정합니다.
    /*
    auto lights = registry.view<Light>();
    for(auto entity : lights) {
        auto& light = lights.get<Light>(entity);
        lightUniformVector = glm::vec4(light.GetPosition(), 1.0f);
        break; // 첫 번째 라이트만 사용
    }
    */

    // 3. 카메라 모드에 따른 조명 타입 결정 (핵심 로직)
    if (activeCamera)
    {
        if (activeCamera->GetProjectionMode() == Camera::Orthogonal)
        {
            // Orthogonal 모드 -> Directional Light (w = 0.0)
            // 방향: 카메라가 바라보는 방향과 동일하게 설정 (Flashlight 효과)
            // 또는 특정 태양광 방향 고정 가능
            glm::vec3 camDir = glm::normalize(activeCamera->GetTarget() - activeCamera->GetEye());

            // 빛의 방향 벡터 (보통 빛이 나아가는 방향 or 빛이 오는 방향, 쉐이더 구현에 따름)
            // 여기서는 빛이 오는 방향(To Light)이 아니라 빛이 가는 방향(Direction)으로 가정하여 넘김
            lightUniformVector = glm::vec4(camDir, 0.0f);
        }
        else
        {
            // Perspective 모드 -> Point Light (w = 1.0)
            // 기존 lightUniformVector의 xyz(위치) 유지, w만 1.0 확인
            lightUniformVector.w = 1.0f;
        }
    }

    // 4. 트랜스폼 업데이트
    {
        auto entites = registry.view<Transform>();
        for (auto& entity : entites)
        {
            auto& transform = entites.get<Transform>(entity);
            if (nullptr == transform.GetParent())
            {
                transform.UpdateAbsoluteTransformMatrix();
            }
        }
    }

    // 5. 불투명(Opaque) 렌더링
    {
        std::map<Shader*, std::vector<Renderable*>> shadermap;
        for (auto& entity : registry.view<Renderable>())
        {
            auto& r = registry.get<Renderable>(entity);
            if (!r.IsUsingAlpha())
                shadermap[r.GetActiveShader()].push_back(&r);
        }

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        RenderRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, lightUniformVector, shadermap);
    }

    // 6. 투명(Transparent) 렌더링
    {
        struct ZRenderable { Renderable* r; float z; };
        std::vector<ZRenderable> transparentObjs;

        for (auto& entity : registry.view<Renderable>())
        {
            auto& r = registry.get<Renderable>(entity);
            if (r.IsUsingAlpha())
            {
                glm::vec3 pos = r.GetVertices().empty() ? glm::vec3(0) : r.GetVertices().at(0);
                float z = glm::length(eye - pos);
                transparentObjs.push_back({ &r, z });
            }
        }

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
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        RenderRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, lightUniformVector, shadermap);

        glDepthMask(GL_TRUE);
    }

    // 7. 디버깅 객체 렌더링 (불투명)
    {
        std::map<Shader*, std::vector<DebuggingRenderable*>> shadermap;
        for (auto& entity : registry.view<DebuggingRenderable>())
        {
            auto& r = registry.get<DebuggingRenderable>(entity);
            if (!r.IsUsingAlpha())
                shadermap[r.GetActiveShader()].push_back(&r);
        }

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

        RenderDebuggingRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, lightUniformVector, shadermap);
    }

    // 8. 디버깅 객체 렌더링 (투명)
    {
        struct ZRenderable { DebuggingRenderable* r; float z; };
        std::vector<ZRenderable> transparentObjs;

        for (auto& entity : registry.view<DebuggingRenderable>())
        {
            auto& r = registry.get<DebuggingRenderable>(entity);
            if (r.IsUsingAlpha())
            {
                glm::vec3 pos = r.GetVertices().empty() ? glm::vec3(0) : r.GetVertices().at(0);
                float z = glm::length(eye - pos);
                transparentObjs.push_back({ &r, z });
            }
        }

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
        glDepthMask(GL_FALSE);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        RenderDebuggingRenderables(frameNo, timeDelta, viewMatrix, perspectiveMatrix, eye, lightUniformVector, shadermap);

        glDepthMask(GL_TRUE);
    }
}