#pragma once

#include <FeatherCommon.h>

class GeometryBuilder
{
public:
	GeometryBuilder();
	~GeometryBuilder();

	static tuple<vector<ui32>, vector<glm::vec3>, vector<glm::vec3>, vector<glm::vec4>, vector<glm::vec2>>
		BuildPlane(f32 width, f32 height, const glm::vec3& center, const glm::vec3& normal, const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

	static tuple<vector<ui32>, vector<glm::vec3>, vector<glm::vec3>, vector<glm::vec4>, vector<glm::vec2>>
		BuildBox(const glm::vec3& center, const glm::vec3& dimension, const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

	static tuple<vector<ui32>, vector<glm::vec3>, vector<glm::vec3>, vector<glm::vec4>, vector<glm::vec2>>
		BuildWiredBox(const glm::vec3& center, const glm::vec3& dimension, const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

	static tuple<vector<ui32>, vector<glm::vec3>, vector<glm::vec3>, vector<glm::vec4>, vector<glm::vec2>>
		BuildSphere(const glm::vec3& center, f32 radius, ui32 horizontalSegments, ui32 verticalSegments, const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
};
