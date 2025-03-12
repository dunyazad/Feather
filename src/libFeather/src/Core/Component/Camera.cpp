#include <Core/Component/Camera.h>

CameraBase::CameraBase(ComponentID id)
	: ComponentBase(id)
{
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

MiniMath::M4 CameraBase::GetViewMatrix() const
{
    return LookAt(eye, target, up);
}

PerspectiveCamera::PerspectiveCamera(ComponentID id)
	: CameraBase(id)
{
}

PerspectiveCamera::~PerspectiveCamera()
{
}

MiniMath::M4 PerspectiveCamera::GetProjectionMatrix() const
{
    float tanHalfFovy = tanf(fovy * 0.5f);

    MiniMath::M4 result = {};

    result.m[0][0] = 1.0f / (aspectRatio * tanHalfFovy);
    result.m[1][1] = 1.0f / tanHalfFovy;
    result.m[2][2] = -(zFar + zNear) / (zFar - zNear);
    result.m[2][3] = -1.0f;
    result.m[3][2] = -(2.0f * zFar * zNear) / (zFar - zNear);
    result.m[3][3] = 0.0f;

    return result;
}

OrthogonalCamera::OrthogonalCamera(ComponentID id)
	: CameraBase(id)
{
}

OrthogonalCamera::~OrthogonalCamera()
{
}

MiniMath::M4 OrthogonalCamera::GetProjectionMatrix() const
{
    MiniMath::M4 result = {};

    result.m[0][0] = 2.0f / (right - left);
    result.m[1][1] = 2.0f / (top - bottom);
    result.m[2][2] = -2.0f / (zFar - zNear);
    result.m[3][0] = -(right + left) / (right - left);
    result.m[3][1] = -(top + bottom) / (top - bottom);
    result.m[3][2] = -(zFar + zNear) / (zFar - zNear);
    result.m[3][3] = 1.0f;

    return result;
}