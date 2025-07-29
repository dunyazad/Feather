#pragma once

#include <FeatherCommon.h>

struct Ray
{
	glm::vec3 origin;
	glm::vec3 direction;
};

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

	virtual void Update(ui32 frameNo, f32 timeDelta) = 0;

	virtual Ray ScreenPointToRay(float mouseX, float mouseY, int screenWidth, int screenHeight) = 0;

	inline bool IsDirty() const { return dirty; }
	inline void SetDirty(bool isDirty) { dirty = isDirty; }

	inline glm::vec3& GetEye() { return eye; }
	inline glm::vec3& GetTarget() { return target; }
	inline glm::vec3& GetUp() { return up; }

	inline void SetEye(const glm::vec3& eye) { this->eye = eye; dirty = true; }
	inline void SetTarget(const glm::vec3& target) { this->target = target; dirty = true; }
	inline void SetUp(const glm::vec3& up) { this->up = up; dirty = true; }

	inline const glm::mat4& GetProjectionMatrix() { return projectionMatrix; }
	inline const glm::mat4& GetViewMatrix() { return viewMatrix; }

	ProjectionMode projectionMode = Perspective;

protected:
	bool dirty = true;

	glm::mat4 projectionMatrix = glm::identity<glm::mat4>();
	glm::mat4 viewMatrix = glm::identity<glm::mat4>();

	glm::vec3 eye = glm::vec3(0.0f, 0.0f, 50.0f);
	glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
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

	inline void SetFOVY(f32 fovy) { this->fovy = fovy; dirty = true; }
	inline void SetAspectRatio(f32 aspectRatio) { this->aspectRatio = aspectRatio; dirty = true; }
	inline void SetNear(f32 zNear) { this->zNear = zNear; dirty = true; }
	inline void SetFar(f32 zFar) { this->zFar = zFar; dirty = true; }

	virtual Ray ScreenPointToRay(float mouseX, float mouseY, int screenWidth, int screenHeight);

protected:
	f32 fovy = 45 * DEG2RAD;
	f32 aspectRatio = 1.0f;
	f32 zNear = 0.1f;
	f32 zFar = 10000.0f;
};

class OrthogonalCamera : public CameraBase
{
public:
	OrthogonalCamera();
	virtual ~OrthogonalCamera();

	virtual void Update(ui32 frameNo, f32 timeDelta);

	virtual Ray ScreenPointToRay(float mouseX, float mouseY, int screenWidth, int screenHeight);

protected:
	f32 left = -1.0f;
	f32 right = 1.0f;
	f32 bottom = -1.0f;
	f32 top = 1.0f;
	f32 zNear = 0.1f;
	f32 zFar = 10000.0f;
};
