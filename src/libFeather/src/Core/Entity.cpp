#include <Core/Entity.h>
#include <Core/Component/Components.h>

Entity::Entity(EntityID id, const string& name)
	: id(id), name(name)
{
}

Entity::~Entity()
{
}

void Entity::AddComponentID(ComponentBase* component)
{
	componentIDs.push_back(component->GetID());
}

void Entity::AddComponentID(ComponentID id)
{
	componentIDs.push_back(id);
}