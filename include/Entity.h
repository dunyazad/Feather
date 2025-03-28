#pragma once

#include <FeatherCommon.h>

class ComponentBase;

class Entity
{
public:
	Entity();
	~Entity();

	inline EntityID GetID() const { return id; }

private:
	static EntityID s_id;
	EntityID id;
};
