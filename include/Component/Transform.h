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

	inline const MiniMath::M4& GetTransformMatrix() const { return transformMatrix; }
	inline void SetTransformMatrix(const MiniMath::M4& m) { transformMatrix = m; }

private:
	Transform* parent = nullptr;
	set<Transform*> children;

	MiniMath::M4 transformMatrix = MiniMath::M4::identity();
};
