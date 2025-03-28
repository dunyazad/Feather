#include <System/EventSystem.h>

#include <FeatherWindow.h>
#include <Feather.h>

vector<EventSystem*> EventSystem::s_instances;

f64 EventSystem::lastMousePositionX = 0.0f;
f64 EventSystem::lastMousePositionY = 0.0f;

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
	auto& dispatcher = Feather.GetDispatcher();
	dispatcher.update();
}

void EventSystem::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	auto& dispatcher = Feather.GetDispatcher();
	dispatcher.enqueue<KeyEvent>({ key, scancode, action, mods });
}

void EventSystem::MousePositionCallback(GLFWwindow* window, f64 xpos, f64 ypos)
{
	auto& dispatcher = Feather.GetDispatcher();
	dispatcher.enqueue<MousePositionEvent>({ xpos, ypos });

	lastMousePositionX = xpos;
	lastMousePositionY = ypos;
}

void EventSystem::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
	auto& dispatcher = Feather.GetDispatcher();
	dispatcher.enqueue<MouseButtonEvent>({ button, action, mods, lastMousePositionX, lastMousePositionY });
}

void EventSystem::MouseWheelCallback(GLFWwindow* window, f64 xoffset, f64 yoffset)
{
	auto& dispatcher = Feather.GetDispatcher();
	dispatcher.enqueue<MouseWheelEvent>({ xoffset, yoffset });
}