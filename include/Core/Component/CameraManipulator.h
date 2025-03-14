#pragma once

#include <FeatherCommon.h>
#include <Core/Component/ComponentBase.h>

class CameraBase;

class CameraManipulatorBase : public ComponentBase
{
public:
	CameraManipulatorBase(ComponentID id);
	virtual ~CameraManipulatorBase() = default;

	inline void SetCamera(CameraBase* camera) { this->camera = camera; }

protected:
	CameraBase* camera = nullptr;
};

class CameraManipulatorOrbit : public CameraManipulatorBase
{
public:
	CameraManipulatorOrbit(ComponentID id);
	~CameraManipulatorOrbit();
	
protected:
	set<i32> pressedKeys;

	f64 lastMousePositionX = UINT32_MAX;
	f64 lastMousePositionY = UINT32_MAX;
	
	f64 lastLButtonPositionX = UINT32_MAX;
	f64 lastLButtonPositionY = UINT32_MAX;
	f64 lastMButtonPositionX = UINT32_MAX;
	f64 lastMButtonPositionY = UINT32_MAX;
	f64 lastRButtonPositionX = UINT32_MAX;
	f64 lastRButtonPositionY = UINT32_MAX;

	bool isLButtonPressed = false;
	bool isMButtonPressed = false;
	bool isRButtonPressed = false;

	f32 azimuth = 0.0f;
	f32 elevation = 0.0f;
	f32 radius = 5.0f;
	f32 mouseSensitivity = 0.2f;
	f32 mouseWheelSensitivity = 0.5f;
};
