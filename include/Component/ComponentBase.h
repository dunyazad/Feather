#pragma once

#include <FeatherCommon.h>

class ComponentBase : public RegisterDerivation<ComponentBase, FeatherObject>, public IEventReceiver
{
public:
	ComponentBase(ComponentID id);
	virtual ~ComponentBase();

	inline ComponentID GetID() const { return id; }

	virtual void Update(ui32 frameNo, f32 timeDelta);

private:
	ComponentID id;
};
