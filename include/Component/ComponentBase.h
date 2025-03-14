#pragma once

#include <FeatherCommon.h>

class ComponentBase : public RegisterDerivation<ComponentBase, FeatherObject>
{
public:
	ComponentBase();
	virtual ~ComponentBase();

	inline ComponentID GetID() const { return id; }

	virtual void Update(ui32 frameNo, f32 timeDelta);

private:
	static ComponentID s_id;
	ComponentID id;
};
