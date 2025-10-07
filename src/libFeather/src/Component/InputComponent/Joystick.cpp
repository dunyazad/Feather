#include <Component/InputComponent/Joystick.h>

Joystick::Joystick(HWND hWnd)
	: hWnd(hWnd)
{
    HRESULT hr;

    hr = DirectInput8Create(GetModuleHandle(NULL), DIRECTINPUT_VERSION, IID_IDirectInput8, (VOID**)&directInput, NULL);
    if (FAILED(hr)) return;

    hr = directInput->EnumDevices(DI8DEVCLASS_GAMECTRL, Joystick::EnumJoysticksCallback, this, DIEDFL_ATTACHEDONLY);
    if (FAILED(hr) || joystickDevice == nullptr)
    {
        directInput->Release();
        directInput = nullptr;
        return;
    }

    hr = joystickDevice->SetDataFormat(&c_dfDIJoystick2);
    if (FAILED(hr))
    {
        joystickDevice->Release();
        joystickDevice = nullptr;
        directInput->Release();
        directInput = nullptr;
        return;
    }

    joystickDevice->SetCooperativeLevel(hWnd, DISCL_EXCLUSIVE | DISCL_BACKGROUND);

    joystickDevice->Acquire();
}

Joystick::~Joystick()
{
    if (joystickDevice)
    {
        joystickDevice->Unacquire();
        joystickDevice->Release();
        joystickDevice = nullptr;
    }
    if (directInput)
    {
        directInput->Release();
        directInput = nullptr;
    }
}

BOOL CALLBACK Joystick::EnumJoysticksCallback(const DIDEVICEINSTANCE* pdidInstance, VOID* pContext)
{
    Joystick* pJoystick = reinterpret_cast<Joystick*>(pContext);
    if(nullptr == pJoystick)
    {
        return DIENUM_CONTINUE;
	}

    HRESULT hr = pJoystick->directInput->CreateDevice(pdidInstance->guidInstance, &pJoystick->joystickDevice, NULL);

    if (FAILED(hr))
    {
        return DIENUM_CONTINUE;
    }

    return DIENUM_STOP;
}

bool Joystick::ReadJoystickRawData(DIJOYSTATE2* outState)
{
    if (joystickDevice == nullptr) return false;

    HRESULT hr = joystickDevice->Poll();
    if (FAILED(hr))
    {
        hr = joystickDevice->Acquire();
        while (hr == DIERR_INPUTLOST)
        {
            hr = joystickDevice->Acquire();
        }
        if (FAILED(hr)) return false;
    }

    hr = joystickDevice->GetDeviceState(sizeof(DIJOYSTATE2), outState);
    if (FAILED(hr)) return false;

    return true;
}

bool Joystick::ReadJoystick(ControllerInputState& controllerInputState)
{
    DIJOYSTATE2 rawState;
	if (!ReadJoystickRawData(&rawState)) return false;

    controllerInputState.AxisX = (rawState.lX / 32767.5f) - 1.0f; // Roll
    controllerInputState.AxisY = (rawState.lY / 32767.5f) - 1.0f; // Pitch
    controllerInputState.AxisZ = (rawState.lZ / 32767.5f) - 1.0f; // Knob VRA
    controllerInputState.RotX = (rawState.lRx / 32767.5f) - 1.0f; // Knob VRB
    controllerInputState.RotY = (rawState.lRy / 32767.5f) - 1.0f; // Throttle Raw
    controllerInputState.RotZ = (rawState.lRz / 32767.5f) - 1.0f; // Yaw

    controllerInputState.AxisX /= 0.708675f;
    controllerInputState.AxisY /= 0.708675f;
    controllerInputState.AxisZ /= 0.708675f;
    controllerInputState.RotX /= 0.708675f;
    controllerInputState.RotY /= 0.708675f;
    controllerInputState.RotZ /= 0.708675f;

    for (int i = 0; i < 16; i++)
    {
        controllerInputState.Buttons[i] = rawState.rgbButtons[i] & 0x80 ? true : false;
	}

	return true;
}

