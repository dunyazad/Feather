#pragma once

#include <System/SystemBase.h>

struct KeyInputEventParameters
{
	i32 keyCode;
	i32 scanCode;
	i32 action;
	i32 mods;
};

struct MouseMoveInputEventParameters
{
	f64 xpos;
	f64 ypos;
};

struct MouseButtonInputEventParameters
{
	i32 button;
	i32 action;
	i32 mods;
};

enum class InputEventType
{
	None,
	Key,
	MouseMove,
	MouseButton,
	NumberOfEventTypes
};

union InputEventParameters
{
	KeyInputEventParameters key;
	MouseMoveInputEventParameters mouseMove;
	MouseButtonInputEventParameters mouseButton;
};

struct InputEvent
{
	InputEventType type;
	InputEventParameters parameters;
};

class InputSystem : public RegisterDerivation<InputSystem, SystemBase>
{
public:
	InputSystem(FeatherWindow* window);
	~InputSystem();

	virtual void Initialize() override;
	virtual void Terminate() override;
	virtual void Update(ui32 frameNo, f32 timeDelta) override;

	void AddKeyPressEventHandler(int key, function<void(GLFWwindow*, KeyInputEventParameters)> handler);
	void AddKeyReleaseEventHandler(int key, function<void(GLFWwindow*, KeyInputEventParameters)> handler);

	void AddMouseMoveEventHandler(function<void(GLFWwindow*, MouseMoveInputEventParameters)> handler);
	void AddMouseButtonPressEventHandler(function<void(GLFWwindow*, MouseButtonInputEventParameters)> handler);
	void AddMouseButtonReleaseEventHandler(function<void(GLFWwindow*, MouseButtonInputEventParameters)> handler);

	static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void MouseMoveCallback(GLFWwindow* window, f64 xpos, f64 ypos);
	static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

private:
	static vector<InputSystem*> s_instances;
	map<int, function<void(GLFWwindow*, KeyInputEventParameters)>> keyPressEventHandlers;
	map<int, function<void(GLFWwindow*, KeyInputEventParameters)>> keyReleaseEventHandlers;

	vector<function<void(GLFWwindow*, MouseMoveInputEventParameters)>> mouseMoveEventHandlers;

	vector<function<void(GLFWwindow*, MouseButtonInputEventParameters)>> mouseButtonPressEventHandlers;
	vector<function<void(GLFWwindow*, MouseButtonInputEventParameters)>> mouseButtonReleaseEventHandlers;

	void OnKeyInput(GLFWwindow* window, KeyInputEventParameters KeyInputEventParameters);
	void OnMouseMoveInput(GLFWwindow* window, MouseMoveInputEventParameters MouseMoveInputEventParameters);
	void OnMouseButtonInput(GLFWwindow* window, MouseButtonInputEventParameters MouseButtonInputEventParameters);
};
