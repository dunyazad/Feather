#pragma once

#include <System/SystemBase.h>

class EventReceiver;

class EventSystem : public SystemBase
{
public:
	EventSystem(FeatherWindow* window);
	~EventSystem();

	virtual void Initialize() override;
	virtual void Terminate() override;
	virtual void Update(ui32 frameNo, f32 timeDelta) override;

	void SubscribeEvent(EventType eventType, IEventReceiver* eventReceiver);
	void UnsubscribeEvent(EventType eventType, IEventReceiver* eventReceiver);

	void DispatchEvent(const Event& event);

private:
	static vector<EventSystem*> s_instances;
	unordered_map<EventType, set<IEventReceiver*>> eventReceivers;

	static void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void MousePositionCallback(GLFWwindow* window, f64 xpos, f64 ypos);
	static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
	static void MouseWheelCallback(GLFWwindow* window, f64 xoffset, f64 yoffset);

	f64 lastMousePositionX = 0;
	f64 lastMousePositionY = 0;
};