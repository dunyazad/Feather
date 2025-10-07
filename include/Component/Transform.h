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

	inline const glm::mat4& GetTransformMatrix() const { return transformMatrix; }
	inline void SetTransformMatrix(const glm::mat4& m) { transformMatrix = m; }

private:
	Transform* parent = nullptr;
	std::set<Transform*> children;

	glm::mat4 transformMatrix = glm::identity<glm::mat4>();
};
