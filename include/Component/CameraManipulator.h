#pragma once

#include <FeatherCommon.h>
#include <vector>
#include <set>
#include <unordered_set>
#include <tuple>

// Forward declaration
class Camera;
struct Event;
struct MousePositionEvent;
struct MouseButtonEvent;
struct MouseWheelEvent;
struct KeyEvent;

class CameraManipulatorBase
{
public:
	CameraManipulatorBase();
	virtual ~CameraManipulatorBase() = default;

	virtual void PushCameraHistory() = 0;
	virtual void PopCameraHistory() = 0;
	virtual void JumpCameraHistory(i32 index) = 0;
	virtual void JumpToPreviousCameraHistory() = 0;
	virtual void JumpToNextCameraHistory() = 0;
	virtual void Reset() = 0;

	inline Camera* GetCamera() const { return camera; }
	inline void SetCamera(Camera* camera) { this->camera = camera; }

protected:
	Camera* camera = nullptr;
};

class CameraManipulatorOrbit : public CameraManipulatorBase
{
public:
	CameraManipulatorOrbit();
	virtual ~CameraManipulatorOrbit();

	// Base implementation stubs (to be implemented if needed)
	virtual void PushCameraHistory() override {}
	virtual void PopCameraHistory() override {}
	virtual void JumpCameraHistory(i32 index) override {}
	virtual void JumpToPreviousCameraHistory() override {}
	virtual void JumpToNextCameraHistory() override {}
	virtual void Reset() override {}

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

protected:
	std::set<i32> pressedKeys;

	f64 lastMousePositionX = 0.0;
	f64 lastMousePositionY = 0.0;

	bool isLButtonPressed = false;
	bool isMButtonPressed = false;
	bool isRButtonPressed = false;

	f32 azimuth = 0.0f;
	f32 elevation = 0.0f;
	f32 radius = 50.0f;
	f32 mouseSensitivity = 0.2f;
	f32 mousePanningSensitivity = 0.1f;
	f32 mouseWheelSensitivity = 0.5f;
};


class CameraManipulatorTrackball : public CameraManipulatorBase
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

	virtual void PushCameraHistory() override;
	virtual void PopCameraHistory() override;
	virtual void JumpCameraHistory(i32 index) override;
	virtual void JumpToPreviousCameraHistory() override;
	virtual void JumpToNextCameraHistory() override;
	virtual void Reset() override;
	void MakeDefault();

	// Override to push history when setting camera
	inline void SetCamera(Camera* camera) { this->camera = camera; if (camera) PushCameraHistory(); }

private:
	float lastMousePositionX = 0.0f;
	float lastMousePositionY = 0.0f;

	bool isLButtonPressed = false;
	bool isMButtonPressed = false;
	bool isRButtonPressed = false;

	f32 radius = 50.0f;
	f32 mouseSensitivity = 0.005f;
	f32 mousePanningSensitivity = 0.01f;
	f32 mouseWheelSensitivity = 0.5f;

	glm::quat cameraRotation = glm::identity<glm::quat>();

	std::unordered_set<int> pressedKeys;

	// History now stores radius as well
	std::vector<std::tuple<glm::vec3, glm::vec3, glm::vec3, f32>> cameraHistory;
	size_t cameraHistoryIndex = 0;
};