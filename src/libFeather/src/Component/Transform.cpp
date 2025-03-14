#include <Component/Transform.h>

Transform::Transform()
	: RegisterDerivation<Transform, ComponentBase>()
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