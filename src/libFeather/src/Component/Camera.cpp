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

    glm::vec4 ray_clip = glm::vec4(x, y, -1.0f, 1.0f);

    glm::mat4 invProj = glm::inverse(projectionMatrix);
    glm::vec4 ray_eye = invProj * ray_clip;
    ray_eye = glm::vec4(ray_eye.x, ray_eye.y, -1.0f, 0.0f);

    glm::mat4 invView = glm::inverse(viewMatrix);
    glm::vec4 ray_world = invView * ray_eye;
    glm::vec3 rayDir = glm::normalize(glm::vec3(ray_world));

    glm::vec3 origin = glm::vec3(glm::inverse(viewMatrix)[3]);

    return { origin, rayDir };
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
