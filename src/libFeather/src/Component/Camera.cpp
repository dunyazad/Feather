#include <Component/Camera.h>
#include <Feather.h>
#include <FeatherWindow.h>

CameraBase::CameraBase()
{
    PushCameraHistory();
}

CameraBase::~CameraBase()
{
}

MiniMath::M4 CameraBase::LookAt(const MiniMath::V3& eye, const MiniMath::V3& target, const MiniMath::V3& up) const
{
    MiniMath::V3 F = MiniMath::normalize(target - eye);
    MiniMath::V3 R = MiniMath::normalize(MiniMath::cross(F, up));
    MiniMath::V3 U = MiniMath::cross(R, F);

    // 결과 행렬 초기화
    MiniMath::M4 result = {};

    result.m[0][0] = R.x;
    result.m[1][0] = R.y;
    result.m[2][0] = R.z;
    result.m[3][0] = MiniMath::dot(-R, eye);

    result.m[0][1] = U.x;
    result.m[1][1] = U.y;
    result.m[2][1] = U.z;
    result.m[3][1] = MiniMath::dot(-U, eye);

    result.m[0][2] = -F.x;
    result.m[1][2] = -F.y;
    result.m[2][2] = -F.z;
    result.m[3][2] = MiniMath::dot(F, eye);

    result.m[0][3] = 0.0f;
    result.m[1][3] = 0.0f;
    result.m[2][3] = 0.0f;
    result.m[3][3] = 1.0f;

    return result;
}

void CameraBase::PushCameraHistory()
{
    cameraHistory.push_back({eye, target, up});
    
    JumpCameraHistory(cameraHistory.size() - 1);
}

void CameraBase::PopCameraHistory()
{
    if (1 >= cameraHistory.size()) return;
    
    cameraHistory.pop_back();

    if (cameraHistoryIndex >= cameraHistory.size())
    {
        JumpCameraHistory(cameraHistory.size() - 1);
    }
}

void CameraBase::JumpCameraHistory(i32 index)
{
    if (index >= cameraHistory.size()) return;

    cameraHistoryIndex = index;
     
    auto [eye, target, up] = cameraHistory[cameraHistoryIndex];

    SetEye(eye);
    SetTarget(target);
    SetUp(up);
}

void CameraBase::JumpToPreviousCameraHistory()
{
    if (0 == cameraHistoryIndex) return;

    JumpCameraHistory(cameraHistoryIndex - 1);
}

void CameraBase::JumpToNextCameraHistory()
{
    if (cameraHistoryIndex >= cameraHistory.size()) return;

    JumpCameraHistory(cameraHistoryIndex + 1);
}

void CameraBase::Reset()
{
    auto [eye, target, up] = cameraHistory.front();
    cameraHistory.clear();
    cameraHistory.push_back({eye, target, up});

    SetEye(eye);
    SetTarget(target);
    SetUp(up);
}

PerspectiveCamera::PerspectiveCamera()
    : CameraBase()
{
    auto window = Feather.GetFeatherWindow();

    aspectRatio = (f32)window->GetWidth() / (f32)window->GetHeight();

    //AddEventHandler(EventType::FrameBufferResize, [&](const Event& event, FeatherObject* object) {
    //    aspectRatio =
    //        (f32)event.frameBufferResizeEvent.width /
    //        (f32)event.frameBufferResizeEvent.height;
    //    });
}

PerspectiveCamera::~PerspectiveCamera()
{
}

//void PerspectiveCamera::Update(ui32 frameNo, f32 timeDelta)
//{
//    if (needToUpdate)
//    {
//        float tanHalfFovy = tanf(fovy * 0.5f);
//
//        projectionMatrix.m[0][0] = 1.0f / (aspectRatio * tanHalfFovy);
//        projectionMatrix.m[1][1] = 1.0f / tanHalfFovy;
//        projectionMatrix.m[2][2] = -(zFar + zNear) / (zFar - zNear);
//        projectionMatrix.m[2][3] = -1.0f;
//        projectionMatrix.m[3][2] = -(2.0f * zFar * zNear) / (zFar - zNear);
//        projectionMatrix.m[3][3] = 0.0f;
//
//        MiniMath::V3 F = MiniMath::normalize(target - eye);
//        MiniMath::V3 R = MiniMath::normalize(MiniMath::cross(F, up));
//        MiniMath::V3 U = MiniMath::cross(R, F);
//
//        viewMatrix.m[0][0] = R.x;
//        viewMatrix.m[1][0] = R.y;
//        viewMatrix.m[2][0] = R.z;
//        viewMatrix.m[3][0] = MiniMath::dot(-R, eye);
//
//        viewMatrix.m[0][1] = U.x;
//        viewMatrix.m[1][1] = U.y;
//        viewMatrix.m[2][1] = U.z;
//        viewMatrix.m[3][1] = MiniMath::dot(-U, eye);
//
//        viewMatrix.m[0][2] = -F.x;
//        viewMatrix.m[1][2] = -F.y;
//        viewMatrix.m[2][2] = -F.z;
//        viewMatrix.m[3][2] = MiniMath::dot(F, eye);
//
//        viewMatrix.m[0][3] = 0.0f;
//        viewMatrix.m[1][3] = 0.0f;
//        viewMatrix.m[2][3] = 0.0f;
//        viewMatrix.m[3][3] = 1.0f;
//
//        needToUpdate = false;
//    }
//}

OrthogonalCamera::OrthogonalCamera()
	: CameraBase()
{
}

OrthogonalCamera::~OrthogonalCamera()
{
}

//void OrthogonalCamera::Update(ui32 frameNo, f32 timeDelta)
//{
//    if (needToUpdate)
//    {
//        projectionMatrix.m[0][0] = 2.0f / (right - left);
//        projectionMatrix.m[1][1] = 2.0f / (top - bottom);
//        projectionMatrix.m[2][2] = -2.0f / (zFar - zNear);
//        projectionMatrix.m[3][0] = -(right + left) / (right - left);
//        projectionMatrix.m[3][1] = -(top + bottom) / (top - bottom);
//        projectionMatrix.m[3][2] = -(zFar + zNear) / (zFar - zNear);
//        projectionMatrix.m[3][3] = 1.0f;
//
//        MiniMath::V3 F = MiniMath::normalize(target - eye);
//        MiniMath::V3 R = MiniMath::normalize(MiniMath::cross(F, up));
//        MiniMath::V3 U = MiniMath::cross(R, F);
//
//        viewMatrix.m[0][0] = R.x;
//        viewMatrix.m[1][0] = R.y;
//        viewMatrix.m[2][0] = R.z;
//        viewMatrix.m[3][0] = MiniMath::dot(-R, eye);
//
//        viewMatrix.m[0][1] = U.x;
//        viewMatrix.m[1][1] = U.y;
//        viewMatrix.m[2][1] = U.z;
//        viewMatrix.m[3][1] = MiniMath::dot(-U, eye);
//
//        viewMatrix.m[0][2] = -F.x;
//        viewMatrix.m[1][2] = -F.y;
//        viewMatrix.m[2][2] = -F.z;
//        viewMatrix.m[3][2] = MiniMath::dot(F, eye);
//
//        viewMatrix.m[0][3] = 0.0f;
//        viewMatrix.m[1][3] = 0.0f;
//        viewMatrix.m[2][3] = 0.0f;
//        viewMatrix.m[3][3] = 1.0f;
//
//        needToUpdate = false;
//    }
//}
