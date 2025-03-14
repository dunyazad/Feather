#include <Entity.h>
#include <Component/Components.h>

EntityID Entity::s_id = 0;

Entity::Entity()
	: id(s_id++)
{
}

Entity::~Entity()
{
}
