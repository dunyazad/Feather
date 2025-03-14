#include <Component/Transform.h>

Transform::Transform(ComponentID id)
	: RegisterDerivation<Transform, ComponentBase>(id)
{
}

Transform::~Transform()
{
}

Transform* Transform::GetParent() const
{
	return parent;
}

void Transform::SetParent(Transform* transform)
{

}

void Transform::AddChild(Transform* child)
{

}

void Transform::RemoveChild(Transform* child)
{

}