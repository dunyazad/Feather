#include <Core/Component/EventReceiver.h>
#include <Feather.h>
#include <Core/System/EventSystem.h>

EventReceiver::EventReceiver(ComponentID id)
	: ComponentBase(id)
{
}

EventReceiver::~EventReceiver()
{
}

void EventReceiver::OnEvent(const Event& event)
{
	for (auto& handler : eventHandlers)
	{
		handler(event);
	}
}

void EventReceiver::AddEventHandler(EventType eventType, function<void(const Event&)> handler)
{
	eventHandlers.push_back(handler);
	auto eventSystem = Feather::GetInstance().GetSystem<EventSystem>();
	if (nullptr != eventSystem)
	{
		eventSystem->SubscribeEvent(eventType, this);
	}
}
