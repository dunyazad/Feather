#pragma once

#include <FeatherCommon.h>

class GeometryBuilder
{
public:
	GeometryBuilder();
	~GeometryBuilder();

	static std::tuple<std::vector<ui32>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec4>, std::vector<glm::vec2>>
		BuildPlane(f32 width, f32 height, ui32 hSegments, ui32 vSegments, const glm::vec3& center, const glm::vec3& normal, const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

	static std::tuple<std::vector<ui32>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec4>, std::vector<glm::vec2>>
		BuildBox(const glm::vec3& center, const glm::vec3& dimension, const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

	static std::tuple<std::vector<ui32>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec4>, std::vector<glm::vec2>>
		BuildWiredBox(const glm::vec3& center, const glm::vec3& dimension, const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));

	static std::tuple<std::vector<ui32>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec4>, std::vector<glm::vec2>>
		BuildSphere(const glm::vec3& center, f32 radius, ui32 horizontalSegments, ui32 verticalSegments, const glm::vec4& color = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
};
