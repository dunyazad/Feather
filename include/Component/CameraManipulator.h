#pragma once

#include <FeatherCommon.h>
#include <Component/ComponentBase.h>

class CameraBase;

class CameraManipulatorBase
{
public:
	CameraManipulatorBase();
	virtual ~CameraManipulatorBase() = default;

	inline CameraBase* SetCamera() const { return camera; }
	inline void SetCamera(CameraBase* camera) { this->camera = camera; }

protected:
	CameraBase* camera = nullptr;
};

class CameraManipulatorOrbit
{
public:
	CameraManipulatorOrbit();
	~CameraManipulatorOrbit();
	
	//virtual void OnEvent(const Event& event) override;

	//void OnMousePosition(const Event& event);
	//void OnMouseButtonPress(const Event& event);
	//void OnMouseButtonRelease(const Event& event);
	//void OnMouseWheel(const Event& event);
	//void OnKeyPress(const Event& event);
	//void OnKeyRelease(const Event& event);

	inline f32 GetAzimuth() { return azimuth; }
	inline void SetAzimuth(f32 azimuth) { this->azimuth = azimuth; }
	inline f32 GetElevation() { return elevation; }
	inline void SetElevation(f32 elevation) { this->elevation = elevation; }
	inline f32 GetRadius() { return radius; }
	inline void SetRadius(f32 radius) { this->radius = radius; }
	inline f32 GetMouseSensitivity() { return mouseSensitivity; }
	inline void SetMouseSensitivity(f32 mouseSensitivity) { this->mouseSensitivity = mouseSensitivity; }
	inline f32 GetMouseWheelSensitivity() { return mouseWheelSensitivity; }
	inline void SetMouseWheelSensitivity(f32 mouseWheelSensitivity) { this->mouseWheelSensitivity = mouseWheelSensitivity; }

	inline CameraBase* SetCamera() const { return camera; }
	inline void SetCamera(CameraBase* camera) { this->camera = camera; }

protected:
	CameraBase* camera = nullptr;

	set<i32> pressedKeys;

	f64 lastMousePositionX = UINT32_MAX;
	f64 lastMousePositionY = UINT32_MAX;
	
	bool isLButtonPressed = false;
	bool isMButtonPressed = false;
	bool isRButtonPressed = false;

	f32 azimuth = 0.0f;
	f32 elevation = 0.0f;
	f32 radius = 5.0f;
	f32 mouseSensitivity = 0.2f;
	f32 mousePanningSensitivity = 0.1f;
	f32 mouseWheelSensitivity = 0.5f;
};


class CameraManipulatorTrackball
{
public:
	CameraManipulatorTrackball();
	virtual ~CameraManipulatorTrackball();

	inline f32 GetRadius() { return radius; }
	inline void SetRadius(f32 radius) { this->radius = radius; }
	inline f32 GetMouseSensitivity() { return mouseSensitivity; }
	inline void SetMouseSensitivity(f32 mouseSensitivity) { this->mouseSensitivity = mouseSensitivity; }
	inline f32 GetMouseWheelSensitivity() { return mouseWheelSensitivity; }
	inline void SetMouseWheelSensitivity(f32 mouseWheelSensitivity) { this->mouseWheelSensitivity = mouseWheelSensitivity; }

	void OnMousePosition(const MousePositionEvent& event);
	void OnMouseButton(const MouseButtonEvent& event);
	void OnMouseWheel(const MouseWheelEvent& event);
	void OnKey(const KeyEvent& event);

	inline CameraBase* GetCamera() const { return camera; }
	inline void SetCamera(CameraBase* camera) { this->camera = camera; }

private:
	CameraBase* camera = nullptr;

	float lastMousePositionX = 0.0f;
	float lastMousePositionY = 0.0f;

	bool isLButtonPressed = false;
	bool isMButtonPressed = false;
	bool isRButtonPressed = false;

	float radius = 5.0f;
	float mouseSensitivity = 0.005f;
	float mousePanningSensitivity = 0.01f;
	f32 mouseWheelSensitivity = 0.5f;

	MiniMath::Quaternion cameraRotation = MiniMath::Quaternion::identity();

	std::unordered_set<int> pressedKeys;
};