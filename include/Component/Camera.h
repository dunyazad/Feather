#pragma once

#include <FeatherCommon.h>

struct ProjectionInfoOrthogonal
{
	f32 left, right, bottom, top, zNear, zFar;
};

struct ProjectionInfoPerspective
{
	f32 fovy, aspectRatio, zNear, zFar;
};

class CameraBase
{
public:
	enum ProjectionMode { Perspective, Orghogonal };

public:
	CameraBase();
	virtual ~CameraBase();

	MiniMath::M4 LookAt(const MiniMath::V3& eye, const MiniMath::V3& target, const MiniMath::V3& up) const;

	virtual void PushCameraHistory();
	virtual void PopCameraHistory();
	virtual void JumpCameraHistory(i32 index);
	virtual void JumpToPreviousCameraHistory();
	virtual void JumpToNextCameraHistory();
	virtual void Reset();

	inline MiniMath::V3& GetEye() { return eye; }
	inline MiniMath::V3& GetTarget() { return target; }
	inline MiniMath::V3& GetUp() { return up; }

	inline void SetEye(const MiniMath::V3& eye) { this->eye = eye; needToUpdate = true; }
	inline void SetTarget(const MiniMath::V3& target) { this->target = target; needToUpdate = true; }
	inline void SetUp(const MiniMath::V3& up) { this->up = up; needToUpdate = true; }

	inline const MiniMath::M4& GetProjectionMatrix() { return projectionMatrix; }
	inline const MiniMath::M4& GetViewMatrix() { return viewMatrix; }

	ProjectionMode projectionMode = Perspective;

protected:
	bool needToUpdate = true;

	MiniMath::M4 projectionMatrix = MiniMath::M4::identity();
	MiniMath::M4 viewMatrix = MiniMath::M4::identity();

	MiniMath::V3 eye = MiniMath::V3(0.0f, 0.0f, 5.0f);
	MiniMath::V3 target = MiniMath::V3(0.0f, 0.0f, 0.0f);
	MiniMath::V3 up = MiniMath::V3(0.0f, 1.0f, 0.0f);

	vector<tuple<MiniMath::V3, MiniMath::V3, MiniMath::V3>> cameraHistory;
	i32 cameraHistoryIndex = 0;
};

class PerspectiveCamera : public CameraBase
{
public:
	PerspectiveCamera();
	virtual ~PerspectiveCamera();

	virtual void Update(ui32 frameNo, f32 timeDelta);

	inline f32 GetFOVY() { return fovy; }
	inline f32 GetAspectRatio() { return aspectRatio; }
	inline f32 GetNear() { return zNear; }
	inline f32 GetFar() { return zFar; }

	inline void SetFOVY(f32 fovy) { this->fovy = fovy; needToUpdate = true; }
	inline void SetAspectRatio(f32 aspectRatio) { this->aspectRatio = aspectRatio; needToUpdate = true; }
	inline void SetNear(f32 zNear) { this->zNear = zNear; needToUpdate = true; }
	inline void SetFar(f32 zFar) { this->zFar = zFar; needToUpdate = true; }

protected:
	f32 fovy = 45 * DEG2RAD;
	f32 aspectRatio = 1.0f;
	f32 zNear = 0.1f;
	f32 zFar = 1000.0f;
};

class OrthogonalCamera : public CameraBase
{
public:
	OrthogonalCamera();
	virtual ~OrthogonalCamera();

	virtual void Update(ui32 frameNo, f32 timeDelta);

protected:
	f32 left = -1.0f;
	f32 right = 1.0f;
	f32 bottom = -1.0f;
	f32 top = 1.0f;
	f32 zNear = 0.1f;
	f32 zFar = 1000.0f;
};
