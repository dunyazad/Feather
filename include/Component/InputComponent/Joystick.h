#pragma once

#include <FeatherCommon.h>

struct ControllerInputState
{
    float AxisX = 0.0f;
    float AxisY = 0.0f;
    float AxisZ = 0.0f;
    float RotX = 0.0f;
	float RotY = 0.0f;
	float RotZ = 0.0f;
	bool Buttons[16] = {
		false, false, false, false,
		false, false, false, false,
		false, false, false, false,
		false, false, false, false };
};

class Joystick
{
public:
	Joystick(HWND hWnd);
	~Joystick();

	static BOOL CALLBACK EnumJoysticksCallback(const DIDEVICEINSTANCE* pdidInstance, VOID* pContext);
	bool ReadJoystickRawData(DIJOYSTATE2* outState);
	bool ReadJoystick(ControllerInputState& controllerInputState);

protected:
	HWND hWnd = nullptr;
	LPDIRECTINPUT8 directInput = nullptr;
	LPDIRECTINPUTDEVICE8 joystickDevice = nullptr;
};
