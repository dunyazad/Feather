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

	inline const glm::mat4& GetLocalTransformMatrix() const { return localTransformMatrix; }
	inline void SetLocalTransformMatrix(const glm::mat4& m) { localTransformMatrix = m; dirty = true; }

	inline const glm::mat4& GetAbsoluteTransformMatrix() const { return absoluteTransformMatrix; }
	inline void SetAbsoluteTransformMatrix(const glm::mat4& m) { absoluteTransformMatrix = m; dirty = true; }

	void UpdateAbsoluteTransformMatrix();

private:
	bool dirty = true;
	Transform* parent = nullptr;
	std::set<Transform*> children;

	glm::mat4 localTransformMatrix = glm::identity<glm::mat4>();
	glm::mat4 absoluteTransformMatrix = glm::identity<glm::mat4>();
};
