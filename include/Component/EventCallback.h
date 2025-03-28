#pragma once

#include <FeatherCommon.h>

template <typename T>
class EventCallback
{
public:
	static void OnEvent(const T& event)
	{
		for (auto& callback : instances[typeid(T)])
		{
			callback->callback(callback->target, event);
		}
	}

	EventCallback(entt::entity target, function<void(entt::entity, const T&)> callback)
		: target(target), callback(callback)
	{
		Feather.GetDispatcher().sink<T>().connect<&OnEvent>();
		instances[typeid(T)].insert(this);
	}

	~EventCallback()
	{
		instances[typeid(T)].erase(this);
	}

protected:
	entt::entity target;
	T event;
	function<void(entt::entity, const T&)> callback;
	
	static unordered_map<type_index, set<EventCallback<T>*>> instances;
};

template <typename T>
unordered_map<type_index, set<EventCallback<T>*>> EventCallback<T>::instances;

