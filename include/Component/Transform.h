#pragma once

#include <FeatherCommon.h>

#include <Component/ComponentBase.h>

class Transform : public RegisterDerivation<Transform , ComponentBase>
{
public:
	Transform(ComponentID id);
	~Transform();

	Transform* GetParent() const;
	void SetParent(Transform* transform);

	void AddChild(Transform* child);
	void RemoveChild(Transform* child);

private:
	Transform* parent = nullptr;
	map<ComponentID, Transform*> children;
};
