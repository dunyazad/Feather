#pragma once

#include <FeatherCommon.h>

class ComponentBase : public IEventReceiver
{
public:
	ComponentBase(ComponentID id);
	virtual ~ComponentBase();

	inline ComponentID GetID() const { return id; }

private:
	ComponentID id;
};
