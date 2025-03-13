#pragma once

#include <FeatherCommon.h>

#include <Core/Component/ComponentBase.h>

struct ProjectionInfoOrthogonal
{
	f32 left, right, bottom, top, zNear, zFar;
};

struct ProjectionInfoPerspective
{
	f32 fovy, aspectRatio, zNear, zFar;
};

class CameraBase : public ComponentBase
{
public:
	enum ProjectionMode { Perspective, Orghogonal };

public:
	CameraBase(ComponentID id);
	virtual ~CameraBase();

	virtual MiniMath::M4 GetProjectionMatrix() const = 0;
	virtual MiniMath::M4 LookAt(const MiniMath::V3& eye, const MiniMath::V3& target, const MiniMath::V3& up) const;
	virtual MiniMath::M4 GetViewMatrix() const;

	MiniMath::V3 eye = MiniMath::V3(5.0f, 2.5f, -5.0f);
	MiniMath::V3 target = MiniMath::V3(0.0f, 0.0f, 0.0f);
	MiniMath::V3 up = MiniMath::V3(0.0f, 0.0f, 1.0f);

	inline MiniMath::V3& GetEye() { return eye; }
	inline MiniMath::V3& GetTarget() { return target; }
	inline MiniMath::V3& GetUp() { return up; }

	ProjectionMode projectionMode = Perspective;
protected:
};

class PerspectiveCamera : public CameraBase
{
public:
	PerspectiveCamera(ComponentID id);
	virtual ~PerspectiveCamera();

	virtual MiniMath::M4 GetProjectionMatrix() const;

protected:
	f32 fovy = 45 * DEG2RAD;
	f32 aspectRatio = 1.0f;
	f32 zNear = 0.1f;
	f32 zFar = 1000.0f;
};

class OrthogonalCamera : public CameraBase
{
public:
	OrthogonalCamera(ComponentID id);
	virtual ~OrthogonalCamera();

	virtual MiniMath::M4 GetProjectionMatrix() const;

protected:
	f32 left = -1.0f;
	f32 right = 1.0f;
	f32 bottom = -1.0f;
	f32 top = 1.0f;
	f32 zNear = 0.1f;
	f32 zFar = 1000.0f;
};
