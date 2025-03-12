#include <Core/System/InputSystem.h>
#include <Core/FeatherWindow.h>

vector<InputSystem*> InputSystem::s_instances;

InputSystem::InputSystem(FeatherWindow* window)
	: SystemBase(window)
{
	s_instances.push_back(this);
}

InputSystem::~InputSystem()
{
}

void InputSystem::Initialize()
{
	glfwSetKeyCallback(window->GetGLFWwindow(), KeyCallback);
}

void InputSystem::Terminate()
{
	keyPressEventHandlers.clear();
	keyReleaseEventHandlers.clear();
}

void InputSystem::Update(ui32 frameNo, f32 timeDelta)
{
}

void InputSystem::AddKeyPressEventHandler(int key, function<void(GLFWwindow*, KeyEvent)> handler)
{
	keyPressEventHandlers[key] = handler;
}

void InputSystem::AddKeyReleaseEventHandler(int key, function<void(GLFWwindow*, KeyEvent)> handler)
{
	keyReleaseEventHandlers[key] = handler;
}

void InputSystem::OnKeyInput(GLFWwindow* window, KeyEvent keyEvent)
{
	if (1 == keyEvent.action)
	{
		if (0 != keyPressEventHandlers.count(keyEvent.keyCode))
		{
			keyPressEventHandlers[keyEvent.keyCode](window, keyEvent);
		}
	}
	else if (0 == keyEvent.action)
	{
		if (0 != keyReleaseEventHandlers.count(keyEvent.keyCode))
		{
			keyReleaseEventHandlers[keyEvent.keyCode](window, keyEvent);
		}
	}
}

void InputSystem::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	for (auto& instance : s_instances)
	{
		if (nullptr != instance)
		{
			instance->OnKeyInput(window, { key, scancode, action, mods });
		}
	}
}
