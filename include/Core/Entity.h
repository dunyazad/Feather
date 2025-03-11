#pragma once

#include <libFeatherCommon.h>

class Entity
{
public:
	Entity(EntityID id);
	~Entity();

	inline EntityID GetID() const { return id; }

private:
	EntityID id;
};
