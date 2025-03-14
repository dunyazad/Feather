#include <System/InputSystem.h>
#include <FeatherWindow.h>

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
	/*glfwSetKeyCallback(window->GetGLFWwindow(), KeyCallback);
	glfwSetCursorPosCallback(window->GetGLFWwindow(), MouseMoveCallback);
	glfwSetMouseButtonCallback(window->GetGLFWwindow(), MouseButtonCallback);*/
}

void InputSystem::Terminate()
{
	keyPressEventHandlers.clear();
	keyReleaseEventHandlers.clear();
}

void InputSystem::Update(ui32 frameNo, f32 timeDelta)
{
}

void InputSystem::AddKeyPressEventHandler(int key, function<void(GLFWwindow*, KeyInputEventParameters)> handler)
{
	keyPressEventHandlers[key] = handler;
}

void InputSystem::AddKeyReleaseEventHandler(int key, function<void(GLFWwindow*, KeyInputEventParameters)> handler)
{
	keyReleaseEventHandlers[key] = handler;
}

void InputSystem::AddMouseMoveEventHandler(function<void(GLFWwindow*, MouseMoveInputEventParameters)> handler)
{
	mouseMoveEventHandlers.push_back(handler);
}

void InputSystem::AddMouseButtonPressEventHandler(function<void(GLFWwindow*, MouseButtonInputEventParameters)> handler)
{
	mouseButtonPressEventHandlers.push_back(handler);
}

void InputSystem::AddMouseButtonReleaseEventHandler(function<void(GLFWwindow*, MouseButtonInputEventParameters)> handler)
{
	mouseButtonReleaseEventHandlers.push_back(handler);
}

void InputSystem::OnKeyInput(GLFWwindow* window, KeyInputEventParameters keyEvent)
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

void InputSystem::OnMouseMoveInput(GLFWwindow* window, MouseMoveInputEventParameters mouseMoveEvent)
{
	for (auto& handler : mouseMoveEventHandlers)
	{
		handler(window, mouseMoveEvent);
	}
}

void InputSystem::OnMouseButtonInput(GLFWwindow* window, MouseButtonInputEventParameters mouseButtonEvent)
{
	if (1 == mouseButtonEvent.action)
	{
		for (auto& handler : mouseButtonPressEventHandlers)
		{
			handler(window, mouseButtonEvent);
		}
	}
	else if (0 == mouseButtonEvent.action)
	{
		for (auto& handler : mouseButtonReleaseEventHandlers)
		{
			handler(window, mouseButtonEvent);
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

void InputSystem::MouseMoveCallback(GLFWwindow* window, f64 xpos, f64 ypos)
{
	for (auto& instance : s_instances)
	{
		if (nullptr != instance)
		{
			instance->OnMouseMoveInput(window, { xpos, ypos });
		}
	}
}

void InputSystem::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
	for (auto& instance : s_instances)
	{
		if (nullptr != instance)
		{
			instance->OnMouseButtonInput(window, { button, action, mods });
		}
	}
}
