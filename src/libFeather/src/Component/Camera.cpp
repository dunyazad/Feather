#include <Component/Camera.h>
#include <Feather.h>
#include <FeatherWindow.h>

CameraBase::CameraBase() {}
CameraBase::~CameraBase() {}

PerspectiveCamera::PerspectiveCamera()
    : CameraBase()
{
    auto window = Feather.GetFeatherWindow();
    aspectRatio = (f32)window->GetWidth() / (f32)window->GetHeight();
}

PerspectiveCamera::~PerspectiveCamera() {}

void PerspectiveCamera::Update(ui32 frameNo, f32 timeDelta)
{
    if (dirty)
    {
        projectionMatrix = glm::perspective(fovy, aspectRatio, zNear, zFar);

        viewMatrix = glm::lookAt(eye, target, up);

        dirty = false;
    }
}

Ray PerspectiveCamera::ScreenPointToRay(float mouseX, float mouseY, int screenWidth, int screenHeight)
{
    float x = (2.0f * mouseX) / (float)screenWidth - 1.0f;
    float y = 1.0f - (2.0f * mouseY) / (float)screenHeight;

    glm::vec4 ray_begin = glm::vec4(x, y, -1.0f, 1.0f);
    glm::vec4 ray_end = glm::vec4(x, y, 1.0f, 1.0f);

    auto inv = glm::inverse(projectionMatrix * viewMatrix);
    auto begin = inv * ray_begin;
    auto end = inv * ray_end;

    begin.w = 1 / begin.w;
    begin.x *= begin.w;
    begin.y *= begin.w;
    begin.z *= begin.w;

    end.w = 1 / end.w;
    end.x *= end.w;
    end.y *= end.w;
    end.z *= end.w;

    auto origin = glm::vec3(begin);
    auto dir = glm::normalize(glm::vec3(end) - glm::vec3(begin));

    return { origin, dir };
}

OrthogonalCamera::OrthogonalCamera() : CameraBase() {}
OrthogonalCamera::~OrthogonalCamera() {}

void OrthogonalCamera::Update(ui32 frameNo, f32 timeDelta)
{
    if (dirty)
    {
        projectionMatrix = glm::ortho(left, right, bottom, top);

        viewMatrix = glm::lookAt(eye, target, up);

        dirty = false;
    }
}

Ray OrthogonalCamera::ScreenPointToRay(float mouseX, float mouseY, int screenWidth, int screenHeight)
{
    float x_ndc = (2.0f * mouseX) / screenWidth - 1.0f;
    float y_ndc = 1.0f - (2.0f * mouseY) / screenHeight;

    glm::vec4 clip(x_ndc, y_ndc, 0.0f, 1.0f);

    glm::mat4 invProj = glm::inverse(projectionMatrix);
    glm::vec4 view = invProj * clip;
    view.w = 1.0f;

    glm::mat4 invView = glm::inverse(viewMatrix);
    glm::vec4 world = invView * view;
    glm::vec3 origin(world.x / world.w, world.y / world.w, world.z / world.w);

    glm::vec3 dir = glm::normalize(target - eye);

    return { origin, dir };
}
