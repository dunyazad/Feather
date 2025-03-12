#include <Core/Entity.h>
#include <Core/Component/Components.h>

Entity::Entity(EntityID id, const string& name)
	: id(id), name(name)
{
}

Entity::~Entity()
{
}

void Entity::AddComponent(ComponentBase* component)
{
	componentIDs.push_back(component->GetID());
}

void Entity::AddComponent(ComponentID id)
{
	componentIDs.push_back(id);
}