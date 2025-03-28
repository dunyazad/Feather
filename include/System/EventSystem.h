#pragma once

#include <FeatherCommon.h>

class FeatherWindow;

class EventSystem
{
public:
	EventSystem(FeatherWindow* window);
	~EventSystem();

	virtual void Initialize();
	virtual void Terminate();
	virtual void Update(ui32 frameNo, f32 timeDelta);

	//void SubscribeEvent(EventType eventType, FeatherObject* eventReceiver);
	//void UnsubscribeEvent(EventType eventType, FeatherObject* eventReceiver);

	//void DispatchEvent(const Event& event);

private:
	static vector<EventSystem*> s_instances;
	FeatherWindow* window = nullptr;

	//unordered_map<EventType, set<FeatherObject*>> eventReceivers;

	static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void MousePositionCallback(GLFWwindow* window, f64 xpos, f64 ypos);
	static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
	static void MouseWheelCallback(GLFWwindow* window, f64 xoffset, f64 yoffset);

	static f64 lastMousePositionX;
	static f64 lastMousePositionY;
};