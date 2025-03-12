#include <FeatherCommon.h>

#include <Core/Component/ComponentBase.h>

class EventReceiver : public ComponentBase
{
public:
	EventReceiver(ComponentID id);
	~EventReceiver();

	void OnEvent(const Event& event);

	void AddEventHandler(EventType eventType, function<void(const Event&)> handler);

private:
	vector<function<void(const Event&)>> eventHandlers;
};
