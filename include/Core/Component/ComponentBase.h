#pragma once

#include <libFeatherCommon.h>

class ComponentBase
{
public:
	ComponentBase(ComponentID id);
	virtual ~ComponentBase();

	inline ComponentID GetID() const { return id; }

private:
	ComponentID id;
};
