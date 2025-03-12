#pragma once

#include <Core/System/SystemBase.h>
#include <Core/Shader.h>

struct KeyEvent
{
	i32 keyCode;
	i32 scanCode;
	i32 action;
	i32 mods;
};

class InputSystem : public SystemBase
{
public:
	InputSystem(FeatherWindow* window);
	~InputSystem();

	virtual void Initialize() override;
	virtual void Terminate() override;
	virtual void Update(ui32 frameNo, f32 timeDelta) override;

	void AddKeyPressEventHandler(int key, function<void(GLFWwindow*, KeyEvent)> handler);
	void AddKeyReleaseEventHandler(int key, function<void(GLFWwindow*, KeyEvent)> handler);

	static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

private:
	static vector<InputSystem*> s_instances;
	map<int, function<void(GLFWwindow*, KeyEvent)>> keyPressEventHandlers;
	map<int, function<void(GLFWwindow*, KeyEvent)>> keyReleaseEventHandlers;

	void OnKeyInput(GLFWwindow* window, KeyEvent keyEvent);
};
