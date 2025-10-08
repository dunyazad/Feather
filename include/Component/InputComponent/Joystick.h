#pragma once

#include <FeatherCommon.h>

class Joystick
{
public:
	Joystick(HWND hWnd);
	~Joystick();

	static BOOL CALLBACK EnumJoysticksCallback(const DIDEVICEINSTANCE* pdidInstance, VOID* pContext);
	bool ReadJoystickRawData(DIJOYSTATE2* outState);
	bool ReadJoystick(JoystickEvent& joystickEvent);

protected:
	HWND hWnd = nullptr;
	LPDIRECTINPUT8 directInput = nullptr;
	LPDIRECTINPUTDEVICE8 joystickDevice = nullptr;
};
