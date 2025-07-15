#include <GeometryBuilder.h>

GeometryBuilder::GeometryBuilder()
{
}

GeometryBuilder::~GeometryBuilder()
{
}

tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
GeometryBuilder::BuildPlane(f32 width, f32 height, const MiniMath::V3& center, const MiniMath::V3& normal, const MiniMath::V4& color)
{
	vector<unsigned int> indices;
	vector<MiniMath::V3> vertices;
	vector<MiniMath::V3> normals;
	vector<MiniMath::V4> colors;
	vector<MiniMath::V2> uvs;

	f32 halfWidth = width * 0.5f;
	f32 halfHeight = height * 0.5f;

	auto r = MiniMath::rotation({ 0.0f, 0.0f, 1.0f }, normal);
	
	vertices.push_back(center + r * MiniMath::V3(-halfWidth, -halfHeight, 0.0f));
	vertices.push_back(center + r * MiniMath::V3( halfWidth, -halfHeight, 0.0f));
	vertices.push_back(center + r * MiniMath::V3( halfWidth,  halfHeight, 0.0f));
	vertices.push_back(center + r * MiniMath::V3(-halfWidth,  halfHeight, 0.0f));
	
	normals.push_back(normal);
	
	colors.push_back(color);

	uvs.push_back(MiniMath::V2(0, 0));
	uvs.push_back(MiniMath::V2(1, 0));
	uvs.push_back(MiniMath::V2(1, 1));
	uvs.push_back(MiniMath::V2(0, 1));

	indices.push_back(0);
	indices.push_back(1);
	indices.push_back(2);
	indices.push_back(2);
	indices.push_back(3);
	indices.push_back(0);

	return make_tuple(indices, vertices, normals, colors, uvs);
}

tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
GeometryBuilder::BuildBox(const MiniMath::V3& center, const MiniMath::V3& dimension, const MiniMath::V4& color)
{
	vector<unsigned int> indices;
	vector<MiniMath::V3> vertices;
	vector<MiniMath::V3> normals;
	vector<MiniMath::V4> colors;
	vector<MiniMath::V2> uvs;

	MiniMath::V3 halfDim = dimension * 0.5f;

	MiniMath::V3 positions[8] = {
		center + MiniMath::V3(-halfDim.x, -halfDim.y, -halfDim.z),
		center + MiniMath::V3(halfDim.x, -halfDim.y, -halfDim.z),
		center + MiniMath::V3(halfDim.x, halfDim.y, -halfDim.z),
		center + MiniMath::V3(-halfDim.x, halfDim.y, -halfDim.z),
		center + MiniMath::V3(-halfDim.x, -halfDim.y, halfDim.z),
		center + MiniMath::V3(halfDim.x, -halfDim.y, halfDim.z),
		center + MiniMath::V3(halfDim.x, halfDim.y, halfDim.z),
		center + MiniMath::V3(-halfDim.x, halfDim.y, halfDim.z)
	};

	MiniMath::V2 uvCoords[4] = {
		MiniMath::V2(0, 0),
		MiniMath::V2(1, 0),
		MiniMath::V2(1, 1),
		MiniMath::V2(0, 1)
	};

	MiniMath::V3 faceNormals[6] = {
		MiniMath::V3(0, 0, -1),
		MiniMath::V3(0, 0, 1),
		MiniMath::V3(-1, 0, 0),
		MiniMath::V3(1, 0, 0),
		MiniMath::V3(0, -1, 0),
		MiniMath::V3(0, 1, 0)
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
		MiniMath::V3 normal = faceNormals[i];

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

tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
GeometryBuilder::BuildWiredBox(const MiniMath::V3& center, const MiniMath::V3& dimension, const MiniMath::V4& color)
{
	vector<ui32> indices;
	vector<MiniMath::V3> vertices;
	vector<MiniMath::V3> normals; // Empty or zero since wireframe has no surface
	vector<MiniMath::V4> colors;
	vector<MiniMath::V2> uvs;     // Empty if not needed for wireframe

	MiniMath::V3 halfDim = dimension * 0.5f;

	// Define 8 corners of the box
	MiniMath::V3 positions[8] = {
		center + MiniMath::V3(-halfDim.x, -halfDim.y, -halfDim.z), // 0
		center + MiniMath::V3(halfDim.x, -halfDim.y, -halfDim.z), // 1
		center + MiniMath::V3(halfDim.x,  halfDim.y, -halfDim.z), // 2
		center + MiniMath::V3(-halfDim.x,  halfDim.y, -halfDim.z), // 3
		center + MiniMath::V3(-halfDim.x, -halfDim.y,  halfDim.z), // 4
		center + MiniMath::V3(halfDim.x, -halfDim.y,  halfDim.z), // 5
		center + MiniMath::V3(halfDim.x,  halfDim.y,  halfDim.z), // 6
		center + MiniMath::V3(-halfDim.x,  halfDim.y,  halfDim.z)  // 7
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

		MiniMath::V3 v0 = positions[edgePairs[i][0]];
		MiniMath::V3 v1 = positions[edgePairs[i][1]];

		vertices.push_back(v0);
		vertices.push_back(v1);

		colors.push_back(color);
		colors.push_back(color);

		normals.push_back(MiniMath::V3(0, 0, 0)); // placeholder
		normals.push_back(MiniMath::V3(0, 0, 0));

		uvs.push_back(MiniMath::V2(0, 0));
		uvs.push_back(MiniMath::V2(0, 0));

		indices.push_back(startIdx);
		indices.push_back(startIdx + 1);
	}

	return make_tuple(indices, vertices, normals, colors, uvs);
}

std::tuple<std::vector<ui32>, std::vector<MiniMath::V3>, std::vector<MiniMath::V3>, std::vector<MiniMath::V4>, std::vector<MiniMath::V2>>
GeometryBuilder::BuildSphere(const MiniMath::V3& center, f32 radius, ui32 horizontalSegments, ui32 verticalSegments, const MiniMath::V4& color)
{
	std::vector<ui32> indices;
	std::vector<MiniMath::V3> vertices;
	std::vector<MiniMath::V3> normals;
	std::vector<MiniMath::V4> colors;
	std::vector<MiniMath::V2> uvs;

	float dTheta = 2.0f * M_PI / horizontalSegments;
	float dPhi = M_PI / verticalSegments;

	for (int i = 0; i <= verticalSegments; ++i)
	{
		float phi = i * dPhi;
		for (int j = 0; j <= horizontalSegments; ++j)
		{
			float theta = j * dTheta;

			MiniMath::V3 normal(
				std::cos(theta) * std::sin(phi),
				std::cos(phi),
				std::sin(theta) * std::sin(phi)
			);

			MiniMath::V3 position = center + radius * normal;

			MiniMath::V2 uv(
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
