#include <GeometryBuilder.h>

GeometryBuilder::GeometryBuilder()
{
}

GeometryBuilder::~GeometryBuilder()
{
}

tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
GeometryBuilder::BuildBox(const MiniMath::V3& center, const MiniMath::V3& dimension)
{
	vector<unsigned int> indices;
	vector<MiniMath::V3> vertices;
	vector<MiniMath::V3> normals;
	vector<MiniMath::V4> colors;
	vector<MiniMath::V2> uvs;

	MiniMath::V3 halfDim = dimension * 0.5f;

	// Define 8 cube vertices (corner positions)
	MiniMath::V3 positions[8] = {
		center + MiniMath::V3(-halfDim.x, -halfDim.y, -halfDim.z), // 0
		center + MiniMath::V3(halfDim.x, -halfDim.y, -halfDim.z),  // 1
		center + MiniMath::V3(halfDim.x, halfDim.y, -halfDim.z),   // 2
		center + MiniMath::V3(-halfDim.x, halfDim.y, -halfDim.z),  // 3
		center + MiniMath::V3(-halfDim.x, -halfDim.y, halfDim.z),  // 4
		center + MiniMath::V3(halfDim.x, -halfDim.y, halfDim.z),   // 5
		center + MiniMath::V3(halfDim.x, halfDim.y, halfDim.z),    // 6
		center + MiniMath::V3(-halfDim.x, halfDim.y, halfDim.z)    // 7
	};

	// Define UVs (same for all faces)
	MiniMath::V2 uvCoords[4] = {
		MiniMath::V2(0, 0),
		MiniMath::V2(1, 0),
		MiniMath::V2(1, 1),
		MiniMath::V2(0, 1)
	};

	// Define face normals
	MiniMath::V3 faceNormals[6] = {
		MiniMath::V3(0, 0, -1), // Front
		MiniMath::V3(0, 0, 1),  // Back
		MiniMath::V3(-1, 0, 0), // Left
		MiniMath::V3(1, 0, 0),  // Right
		MiniMath::V3(0, -1, 0), // Bottom
		MiniMath::V3(0, 1, 0)   // Top
	};

	// Define the 6 faces (two triangles per face, in CCW order)
	unsigned int faceIndices[6][6] = {
		{0, 1, 2, 2, 3, 0}, // Front
		{5, 4, 7, 7, 6, 5}, // Back
		{4, 0, 3, 3, 7, 4}, // Left
		{1, 5, 6, 6, 2, 1}, // Right
		{4, 5, 1, 1, 0, 4}, // Bottom
		{3, 2, 6, 6, 7, 3}  // Top
	};

	// Generate vertices, indices, and attributes
	for (int i = 0; i < 6; ++i) // For each face
	{
		MiniMath::V3 normal = faceNormals[i];

		for (int j = 0; j < 6; ++j) // Each face has 6 indices (2 triangles)
		{
			unsigned int index = faceIndices[i][j];

			indices.push_back(vertices.size()); // Use sequential indexing
			vertices.push_back(positions[index]);
			normals.push_back(normal);
			colors.push_back(MiniMath::V4(1.0f, 1.0f, 1.0f, 1.0f));
			uvs.push_back(uvCoords[j % 4]); // Assign UVs
		}
	}

	return make_tuple(indices, vertices, normals, colors, uvs);
}

std::tuple<std::vector<ui32>, std::vector<MiniMath::V3>, std::vector<MiniMath::V3>, std::vector<MiniMath::V4>, std::vector<MiniMath::V2>>
GeometryBuilder::BuildSphere(const MiniMath::V3& center, f32 radius, ui32 horizontalSegments, ui32 verticalSegments)
{
	std::vector<ui32> indices;
	std::vector<MiniMath::V3> vertices;
	std::vector<MiniMath::V3> normals;
	std::vector<MiniMath::V4> colors;
	std::vector<MiniMath::V2> uvs;

	float dTheta = 2.0f * M_PI / horizontalSegments;
	float dPhi = M_PI / verticalSegments;

	// Generate vertices
	for (int i = 0; i <= verticalSegments; ++i)
	{
		float phi = i * dPhi;
		for (int j = 0; j <= horizontalSegments; ++j)
		{
			float theta = j * dTheta;

			// Compute vertex position
			MiniMath::V3 normal(
				std::cos(theta) * std::sin(phi),
				std::cos(phi),
				std::sin(theta) * std::sin(phi)
			);

			MiniMath::V3 position = center + radius * normal;

			// Compute UV coordinates
			MiniMath::V2 uv(
				static_cast<float>(j) / horizontalSegments,
				static_cast<float>(i) / verticalSegments
			);

			// Store vertex data
			vertices.push_back(position);
			normals.push_back(normal);
			uvs.push_back(uv);
			colors.push_back(MiniMath::V4(1.0f, 1.0f, 1.0f, 1.0f)); // White color
		}
	}

	// Generate indices (triangles)
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
