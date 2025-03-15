#include <System/EventSystem.h>

#include <FeatherWindow.h>

vector<EventSystem*> EventSystem::s_instances;

EventSystem::EventSystem(FeatherWindow* window)
	: RegisterDerivation<EventSystem, SystemBase>(window)
{
	s_instances.push_back(this);
}

EventSystem::~EventSystem()
{
}

void EventSystem::Initialize()
{
	glfwSetKeyCallback(window->GetGLFWwindow(), KeyCallback);
	glfwSetCursorPosCallback(window->GetGLFWwindow(), MousePositionCallback);
	glfwSetMouseButtonCallback(window->GetGLFWwindow(), MouseButtonCallback);
	glfwSetScrollCallback(window->GetGLFWwindow(), MouseWheelCallback);
}

void EventSystem::Terminate()
{
}

void EventSystem::Update(ui32 frameNo, f32 timeDelta)
{
}

void EventSystem::SubscribeEvent(EventType eventType, FeatherObject* eventReceiver)
{
	eventReceivers[eventType].insert(eventReceiver);
}

void EventSystem::UnsubscribeEvent(EventType eventType, FeatherObject* eventReceiver)
{
	eventReceivers[eventType].erase(eventReceiver);
}

void EventSystem::DispatchEvent(const Event& event)
{
	if (0 != eventReceivers.count(event.type))
	{
		for (auto& receiver : eventReceivers[event.type])
		{
			receiver->OnEvent(event);
		}
	}
}

void EventSystem::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (0 == action)
	{
		Event event;
		event.type = EventType::KeyRelease;
		event.keyEvent.keyCode = key;
		event.keyEvent.scanCode = scancode;
		event.keyEvent.mods = mods;

		for (auto& instance : s_instances)
		{
			instance->DispatchEvent(event);
		}
	}
	else if (1 == action)
	{
		Event event;
		event.type = EventType::KeyPress;
		event.keyEvent.keyCode = key;
		event.keyEvent.scanCode = scancode;
		event.keyEvent.mods = mods;

		for (auto& instance : s_instances)
		{
			instance->DispatchEvent(event);
		}
	}
}

void EventSystem::MousePositionCallback(GLFWwindow* window, f64 xpos, f64 ypos)
{
	Event event;
	event.type = EventType::MousePosition;
	event.mousePositionEvent.xpos = xpos;
	event.mousePositionEvent.ypos = ypos;

	for (auto& instance : s_instances)
	{
		instance->lastMousePositionX = xpos;
		instance->lastMousePositionY = ypos;

		instance->DispatchEvent(event);
	}
}

void EventSystem::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
	if (0 == action)
	{
		Event event;
		event.type = EventType::MouseButtonRelease;
		event.mouseButtonEvent.button = button;
		event.mouseButtonEvent.mods = mods;

		for (auto& instance : s_instances)
		{
			event.mouseButtonEvent.xpos = instance->lastMousePositionX;
			event.mouseButtonEvent.ypos = instance->lastMousePositionY;

			instance->DispatchEvent(event);
		}
	}
	else if (1 == action)
	{
		Event event;
		event.type = EventType::MouseButtonPress;
		event.mouseButtonEvent.button = button;
		event.mouseButtonEvent.mods = mods;

		for (auto& instance : s_instances)
		{
			event.mouseButtonEvent.xpos = instance->lastMousePositionX;
			event.mouseButtonEvent.ypos = instance->lastMousePositionY;

			instance->DispatchEvent(event);
		}
	}
}

void EventSystem::MouseWheelCallback(GLFWwindow* window, f64 xoffset, f64 yoffset)
{
	Event event;
	event.type = EventType::MouseWheel;
	event.mouseWheelEvent.xoffset = xoffset;
	event.mouseWheelEvent.yoffset = yoffset;

	for (auto& instance : s_instances)
	{
		instance->DispatchEvent(event);
	}
}