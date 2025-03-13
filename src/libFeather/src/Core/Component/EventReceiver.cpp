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
	if (0 != eventHandlers.count(event.type))
	{
		for (auto& handler : eventHandlers[event.type])
		{
			handler(event);
		}
	}
}

void EventReceiver::AddEventHandler(EventType eventType, function<void(const Event&)> handler)
{
	eventHandlers[eventType].push_back(handler);
	auto eventSystem = Feather::GetInstance().GetSystem<EventSystem>();
	if (nullptr != eventSystem)
	{
		eventSystem->SubscribeEvent(eventType, this);
	}
}
