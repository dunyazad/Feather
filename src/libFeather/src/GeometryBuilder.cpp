#include <GeometryBuilder.h>

GeometryBuilder::GeometryBuilder()
{
}

GeometryBuilder::~GeometryBuilder()
{
}

std::tuple<std::vector<ui32>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec4>, std::vector<glm::vec2>>
GeometryBuilder::BuildPlane(f32 width, f32 height, ui32 hSegments, ui32 vSegments, const glm::vec3& center, const glm::vec3& normal, const glm::vec4& color)
{
	std::vector<unsigned int> indices;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> normals;
	std::vector<glm::vec4> colors;
	std::vector<glm::vec2> uvs;

	if (hSegments < 1 || vSegments < 1) {
		return {};
	}

	f32 halfWidth = width * 0.5f;
	f32 halfHeight = height * 0.5f;

	auto r = glm::rotation({ 0.0f, 0.0f, 1.0f }, normal);

	for (ui32 j = 0; j <= vSegments; ++j) {
		for (ui32 i = 0; i <= hSegments; ++i) {
			f32 x = (static_cast<f32>(i) / hSegments) * width - halfWidth;
			f32 y = (static_cast<f32>(j) / vSegments) * height - halfHeight;
			glm::vec3 localPos(x, y, 0.0f);

			vertices.push_back(center + r * localPos);
			normals.push_back(normal);
			colors.push_back(color);

			uvs.push_back(glm::vec2(static_cast<f32>(i) / hSegments, static_cast<f32>(j) / vSegments));
		}
	}

	for (ui32 j = 0; j < vSegments; ++j) {
		for (ui32 i = 0; i < hSegments; ++i) {
			ui32 row1 = j * (hSegments + 1);
			ui32 row2 = (j + 1) * (hSegments + 1);

			indices.push_back(row1 + i);
			indices.push_back(row1 + i + 1);
			indices.push_back(row2 + i);

			indices.push_back(row2 + i);
			indices.push_back(row1 + i + 1);
			indices.push_back(row2 + i + 1);
		}
	}

	return make_tuple(indices, vertices, normals, colors, uvs);
}

std::tuple<std::vector<ui32>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec4>, std::vector<glm::vec2>>
GeometryBuilder::BuildBox(const glm::vec3& center, const glm::vec3& dimension, const glm::vec4& color)
{
	std::vector<unsigned int> indices;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> normals;
	std::vector<glm::vec4> colors;
	std::vector<glm::vec2> uvs;

	glm::vec3 halfDim = dimension * 0.5f;

	glm::vec3 positions[8] = {
		center + glm::vec3(-halfDim.x, -halfDim.y, -halfDim.z),
		center + glm::vec3(halfDim.x, -halfDim.y, -halfDim.z),
		center + glm::vec3(halfDim.x, halfDim.y, -halfDim.z),
		center + glm::vec3(-halfDim.x, halfDim.y, -halfDim.z),
		center + glm::vec3(-halfDim.x, -halfDim.y, halfDim.z),
		center + glm::vec3(halfDim.x, -halfDim.y, halfDim.z),
		center + glm::vec3(halfDim.x, halfDim.y, halfDim.z),
		center + glm::vec3(-halfDim.x, halfDim.y, halfDim.z)
	};

	glm::vec2 uvCoords[4] = {
		glm::vec2(0, 0),
		glm::vec2(1, 0),
		glm::vec2(1, 1),
		glm::vec2(0, 1)
	};

	glm::vec3 faceNormals[6] = {
		glm::vec3(0, 0, -1),
		glm::vec3(0, 0, 1),
		glm::vec3(-1, 0, 0),
		glm::vec3(1, 0, 0),
		glm::vec3(0, -1, 0),
		glm::vec3(0, 1, 0)
	};

	unsigned int faceIndices[6][6] = {
		{0, 1, 2, 2, 3, 0},
		{5, 4, 7, 7, 6, 5},
		{4, 0, 3, 3, 7, 4},
		{1, 5, 6, 6, 2, 1},
		{4, 5, 1, 1, 0, 4},
		{3, 2, 6, 6, 7, 3}
	};

	for (int i = 0; i < 6; ++i)
	{
		glm::vec3 normal = faceNormals[i];

		for (int j = 0; j < 6; ++j)
		{
			unsigned int index = faceIndices[i][j];

			indices.push_back(vertices.size());
			vertices.push_back(positions[index]);
			normals.push_back(normal);
			colors.push_back(color);
			uvs.push_back(uvCoords[j % 4]);
		}
	}

	return make_tuple(indices, vertices, normals, colors, uvs);
}

std::tuple<std::vector<ui32>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec4>, std::vector<glm::vec2>>
GeometryBuilder::BuildWiredBox(const glm::vec3& center, const glm::vec3& dimension, const glm::vec4& color)
{
	std::vector<ui32> indices;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> normals; // Empty or zero since wireframe has no surface
	std::vector<glm::vec4> colors;
	std::vector<glm::vec2> uvs;     // Empty if not needed for wireframe

	glm::vec3 halfDim = dimension * 0.5f;

	// Define 8 corners of the box
	glm::vec3 positions[8] = {
		center + glm::vec3(-halfDim.x, -halfDim.y, -halfDim.z), // 0
		center + glm::vec3(halfDim.x, -halfDim.y, -halfDim.z), // 1
		center + glm::vec3(halfDim.x,  halfDim.y, -halfDim.z), // 2
		center + glm::vec3(-halfDim.x,  halfDim.y, -halfDim.z), // 3
		center + glm::vec3(-halfDim.x, -halfDim.y,  halfDim.z), // 4
		center + glm::vec3(halfDim.x, -halfDim.y,  halfDim.z), // 5
		center + glm::vec3(halfDim.x,  halfDim.y,  halfDim.z), // 6
		center + glm::vec3(-halfDim.x,  halfDim.y,  halfDim.z)  // 7
	};

	// Define the 12 edges of a box using index pairs
	const int edgePairs[12][2] = {
		{0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom face
		{4, 5}, {5, 6}, {6, 7}, {7, 4}, // top face
		{0, 4}, {1, 5}, {2, 6}, {3, 7}  // vertical edges
	};

	for (int i = 0; i < 12; ++i)
	{
		const int startIdx = vertices.size(); // Each edge is 2 vertices

		glm::vec3 v0 = positions[edgePairs[i][0]];
		glm::vec3 v1 = positions[edgePairs[i][1]];

		vertices.push_back(v0);
		vertices.push_back(v1);

		colors.push_back(color);
		colors.push_back(color);

		normals.push_back(glm::vec3(0, 0, 0)); // placeholder
		normals.push_back(glm::vec3(0, 0, 0));

		uvs.push_back(glm::vec2(0, 0));
		uvs.push_back(glm::vec2(0, 0));

		indices.push_back(startIdx);
		indices.push_back(startIdx + 1);
	}

	return make_tuple(indices, vertices, normals, colors, uvs);
}

std::tuple<std::vector<ui32>, std::vector<glm::vec3>, std::vector<glm::vec3>, std::vector<glm::vec4>, std::vector<glm::vec2>>
GeometryBuilder::BuildSphere(const glm::vec3& center, f32 radius, ui32 horizontalSegments, ui32 verticalSegments, const glm::vec4& color)
{
	std::vector<ui32> indices;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> normals;
	std::vector<glm::vec4> colors;
	std::vector<glm::vec2> uvs;

	float dTheta = 2.0f * M_PI / horizontalSegments;
	float dPhi = M_PI / verticalSegments;

	for (int i = 0; i <= verticalSegments; ++i)
	{
		float phi = i * dPhi;
		for (int j = 0; j <= horizontalSegments; ++j)
		{
			float theta = j * dTheta;

			glm::vec3 normal(
				std::cos(theta) * std::sin(phi),
				std::cos(phi),
				std::sin(theta) * std::sin(phi)
			);

			glm::vec3 position = center + radius * normal;

			glm::vec2 uv(
				static_cast<float>(j) / horizontalSegments,
				static_cast<float>(i) / verticalSegments
			);

			vertices.push_back(position);
			normals.push_back(normal);
			uvs.push_back(uv);
			colors.push_back(color);
		}
	}

	for (int i = 0; i < verticalSegments; ++i)
	{
		for (int j = 0; j < horizontalSegments; ++j)
		{
			int current = i * (horizontalSegments + 1) + j;
			int next = current + horizontalSegments + 1;

			indices.push_back(current);
			indices.push_back(next);
			indices.push_back(current + 1);

			indices.push_back(current + 1);
			indices.push_back(next);
			indices.push_back(next + 1);
		}
	}

	return std::make_tuple(indices, vertices, normals, colors, uvs);
}
