#pragma once

#include <FeatherCommon.h>

class Camera
{
public:
	enum ProjectionMode
	{
		Perspective,
		Orthogonal
	};

	class PerspectiveSettings
	{
	public:
		inline bool IsDirty() const { return dirty; }

		inline f32 GetFovy() const { return fovy * (f32)RAD2DEG; }
		inline void SetFovy(f32 fovy) { this->fovy = fovy; dirty = true; }
		
		inline f32 GetAspectRatio() const { return aspectRatio; }
		inline void SetAspectRatio(f32 aspectRatio) { this->aspectRatio = aspectRatio; dirty = true; }

		inline f32 GetZNear() const { return zNear; }
		inline void SetZNear(f32 zNear) { this->zNear = zNear; dirty = true; }

		inline f32 GetZFar() const { return zFar; }
		inline void SetZFar(f32 zFar) { this->zFar = zFar; dirty = true; }

	private:
		bool dirty = true;
		f32 fovy = 45.0f * (f32)DEG2RAD;
		f32 aspectRatio = 1.0f;
		f32 zNear = 0.01f;
		f32 zFar = 1000.0f;
	};

	// 직교 투영용 설정값
	class OrthogonalSettings
	{
	public:
		inline bool IsDirty() const { return dirty; }
		
		inline f32 GetLeft() const { return left; }
		inline void SetLeft(f32 left) { this->left = left; dirty = true; }
		
		inline f32 GetRight() const { return right; }
		inline void SetRight(f32 right) { this->right = right; dirty = true; }
		
		inline f32 GetBottom() const { return bottom; }
		inline void SetBottom(f32 bottom) { this->bottom = bottom; dirty = true; }
		
		inline f32 GetTop() const { return top; }
		inline void SetTop(f32 top) { this->top = top; dirty = true; }
		
		inline f32 GetZNear() const { return zNear; }
		inline void SetZNear(f32 zNear) { this->zNear = zNear; dirty = true; }

		inline f32 GetZFar() const { return zFar; }
		inline void SetZFar(f32 zFar) { this->zFar = zFar; dirty = true; }

	private:
		bool dirty = true;
		f32 left = -10.0f;
		f32 right = 10.0f;
		f32 bottom = -10.0f;
		f32 top = 10.0f;
		f32 zNear = -1000.0f;
		f32 zFar = 1000.0f;
	};

public:
	Camera();
	~Camera();

	void Update(ui32 frameNo, f32 timeDelta);
	Ray ScreenPointToRay(float mouseX, float mouseY, int screenWidth, int screenHeight);

	void SetProjectionMode(ProjectionMode mode);
	inline ProjectionMode GetProjectionMode() const { return mode; }

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

	inline PerspectiveSettings& GetPerspectiveSettings() { dirty = true; return perspectiveSettings; }
	inline OrthogonalSettings& GetOrthogonalSettings() { dirty = true; return orthogonalSettings; }

private:
	bool dirty = true;
	ProjectionMode mode = Perspective;

	PerspectiveSettings perspectiveSettings;
	OrthogonalSettings orthogonalSettings;

	glm::vec3 eye = glm::vec3(0.0f, 0.0f, 50.0f);
	glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);
	glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);

	glm::mat4 projectionMatrix = glm::identity<glm::mat4>();
	glm::mat4 viewMatrix = glm::identity<glm::mat4>();
};