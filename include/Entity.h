#pragma once

#include <FeatherCommon.h>

class ComponentBase;

class Entity : public RegisterDerivation<Entity, FeatherObject>
{
public:
	Entity(EntityID id, const string& name = "");
	~Entity();

	inline EntityID GetID() const { return id; }
	inline const string& GetName() const { return name; }

private:
	string name = "";
	EntityID id;
};
