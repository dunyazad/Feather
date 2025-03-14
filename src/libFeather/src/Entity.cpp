#include <Entity.h>
#include <Component/Components.h>

Entity::Entity(EntityID id, const string& name)
	: id(id), name(name)
{
}

Entity::~Entity()
{
}
