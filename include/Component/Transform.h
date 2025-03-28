#pragma once

#include <FeatherCommon.h>

class Transform
{
public:
	Transform();
	~Transform();

	Transform* GetParent() const;
	void SetParent(Transform* transform);

	void AddChild(Transform* child);
	void RemoveChild(Transform* child);

private:
	Transform* parent = nullptr;
	set<Transform*> children;
};
