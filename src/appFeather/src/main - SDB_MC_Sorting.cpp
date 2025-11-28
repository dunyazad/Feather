#include <libFeather.h>
#include <unordered_map>

struct Configuration
{
	const float voxelSize = 0.2f;
	const int sdfOffset = 1;
	glm::vec3 filterMin = glm::vec3(-20.0f, -20.0f, -20.0f);
	glm::vec3 filterMax = glm::vec3(20.0f, 20.0f, 20.0f);
} Configuration;

using VD = VisualDebugging;
using namespace libRxTx;

const int VpB = 8;
const int VpBHalf = 4;

extern int edgeTable[256];
extern int triTable[256][16];

struct Voxel
{
	bool valid = false;
	float signedDistance = 0.0f;
	float weight = 0.0f;
	glm::vec3 normal = glm::zero<glm::vec3>();
	glm::vec3 color = glm::zero<glm::vec3>();
};

struct DataBlock
{
	glm::vec3 blockMin = glm::zero<glm::vec3>();
	Voxel voxels[VpB * VpB * VpB];

	void Initialize()
	{
		memset(voxels, 0, sizeof(Voxel) * VpB * VpB * VpB);
		//for (size_t i = 0; i < VpB * VpB * VpB; i++)
		//{
		//	voxels[i].signedDistance = FLT_MAX;
		//}
	}
};

typedef uint64_t DataBlockKey;

struct SparseDataBlock
{
	float voxelSize = Configuration.voxelSize;
	glm::vec3 gridOrigin = glm::zero<glm::vec3>();
	std::unordered_map<DataBlockKey, DataBlock> dataBlocks;

	Voxel* GetVoxelByIndex(int gx, int gy, int gz)
	{
		int bx = (int)floor((float)gx / VpB);
		int by = (int)floor((float)gy / VpB);
		int bz = (int)floor((float)gz / VpB);

		float blockSize = voxelSize * VpB;
		glm::vec3 blockMin = gridOrigin + glm::vec3((float)bx * blockSize, (float)by * blockSize, (float)bz * blockSize);

		auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSize);

		auto it = dataBlocks.find(key);
		if (it == dataBlocks.end())
		{
			return nullptr;
		}

		int lx = gx % VpB;
		if (lx < 0) lx += VpB;
		int ly = gy % VpB;
		if (ly < 0) ly += VpB;
		int lz = gz % VpB;
		if (lz < 0) lz += VpB;

		return &it->second.voxels[lz * VpB * VpB + ly * VpB + lx];
	}

	void FromPLY(const std::string& filename)
	{
		TS(PLYLoading);
		PLYFormat ply;
		ply.Deserialize(filename);
		ply.FilterWithinAABB(
			Configuration.filterMin.x, Configuration.filterMin.y, Configuration.filterMin.z,
			Configuration.filterMax.x, Configuration.filterMax.y, Configuration.filterMax.z);
		TE(PLYLoading);

		if (ply.GetPoints().empty())
		{
			return;
		}

		auto [minx, miny, minz] = ply.GetAABBMin();
		glm::vec3 aabbMin(minx - 1.0f, miny - 1.0f, minz - 1.0f);
		auto [maxx, maxy, maxz] = ply.GetAABBMax();
		glm::vec3 aabbMax(maxx + 1.0f, maxy + 1.0f, maxz + 1.0f);

		FromPoints(
			(glm::vec3*)ply.GetPoints().data(),
			(glm::vec3*)ply.GetNormals().data(),
			(glm::vec3*)ply.GetColors().data(),
			(unsigned int)ply.GetPoints().size() / 3,
			aabbMin, aabbMax);
	}

	void FromPoints(glm::vec3* points, glm::vec3* normals, glm::vec3* colors, unsigned int numberOfPoints, const glm::vec3& aabbMin, const glm::vec3& aabbMax)
	{
		float blockSize = voxelSize * VpB;
		gridOrigin.x = std::floor(aabbMin.x / blockSize) * blockSize;
		gridOrigin.y = std::floor(aabbMin.y / blockSize) * blockSize;
		gridOrigin.z = std::floor(aabbMin.z / blockSize) * blockSize;

		TS(Occupy);

		float truncDist = std::max(voxelSize * 4.0f, 0.15f);
		//int searchRange = (int)std::ceil(truncDist / voxelSize);
		int searchRange = Configuration.sdfOffset;

		for (size_t i = 0; i < numberOfPoints; i++)
		{
			auto p = points[i];
			glm::vec3 n = (normals) ? normals[i] : glm::vec3(0, 1, 0);
			glm::vec3 c = (colors) ? colors[i] : glm::vec3(1, 1, 1);

			if (c.r > 1.0f || c.g > 1.0f || c.b > 1.0f)
			{
				c /= 255.0f;
			}

			glm::vec3 vecFromOrigin = p - gridOrigin;
			int centerGx = (int)std::floor(vecFromOrigin.x / voxelSize);
			int centerGy = (int)std::floor(vecFromOrigin.y / voxelSize);
			int centerGz = (int)std::floor(vecFromOrigin.z / voxelSize);

			for (int dz = -searchRange; dz <= searchRange; ++dz)
			{
				for (int dy = -searchRange; dy <= searchRange; ++dy)
				{
					for (int dx = -searchRange; dx <= searchRange; ++dx)
					{
						int gx = centerGx + dx;
						int gy = centerGy + dy;
						int gz = centerGz + dz;

						if (gx < 0 || gy < 0 || gz < 0) continue;

						glm::vec3 voxelCenter = gridOrigin + glm::vec3(
							(gx + 0.5f) * voxelSize,
							(gy + 0.5f) * voxelSize,
							(gz + 0.5f) * voxelSize
						);

						float dist = glm::distance(p, voxelCenter);

						if (dist > truncDist) continue;

						float weight = 1.0f - (dist / truncDist);

						float sdf = glm::dot(voxelCenter - p, n);

						if (sdf < -truncDist) sdf = -truncDist;
						if (sdf > truncDist) sdf = truncDist;

						int bx = (int)std::floor((float)gx / VpB);
						int by = (int)std::floor((float)gy / VpB);
						int bz = (int)std::floor((float)gz / VpB);

						float blockSizeCurr = voxelSize * VpB;
						glm::vec3 blockMin = gridOrigin + glm::vec3(bx * blockSizeCurr, by * blockSizeCurr, bz * blockSizeCurr);

						auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSizeCurr);

						if (dataBlocks.find(key) == dataBlocks.end())
						{
							dataBlocks[key].Initialize();
							dataBlocks[key].blockMin = blockMin;
						}

						int lx = gx % VpB;
						int ly = gy % VpB;
						int lz = gz % VpB;
						Voxel& voxel = dataBlocks[key].voxels[lz * VpB * VpB + ly * VpB + lx];

						if (voxel.weight <= 0.0001f)
						{
							voxel.signedDistance = sdf;
							voxel.color = c;
							voxel.normal = n;
							voxel.weight = weight;
							voxel.valid = true;
						}
						else
						{
							float newW = voxel.weight + weight;
							voxel.signedDistance = (voxel.signedDistance * voxel.weight + sdf * weight) / newW;
							voxel.color = (voxel.color * voxel.weight + c * weight) / newW;

							voxel.normal = (voxel.normal * voxel.weight + n * weight) / newW;
							//voxel.normal = glm::normalize(voxel.normal);

							voxel.weight = newW;
						}
					}
				}
			}
		}
		TE(Occupy);
	}
};

//struct Point3D
//{
//	float x, y, z;
//	bool operator<(const Point3D& other) const
//	{
//		return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
//	}
//};

struct Triangle
{
	glm::vec3 v[3];
	glm::vec3 n[3];
	glm::vec3 c[3];
};

struct MeshGenerator
{
	std::vector<Triangle> triangles;
	std::vector<std::pair<glm::vec3, glm::vec3>> holeEdges;

	const glm::vec3 cornerOffsets[8] = {
		{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1},
		{0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}
	};

	struct GridKey
	{
		int x, y, z;
		bool operator==(const GridKey& other) const
		{
			return x == other.x && y == other.y && z == other.z;
		}
	};

	struct GridKeyHash
	{
		std::size_t operator()(const GridKey& k) const
		{
			return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
		}
	};

	glm::vec3 VertexInterp(float isolevel, glm::vec3 p1, glm::vec3 p2, float val1, float val2)
	{
		if (std::abs(val1 - val2) < 0.00001f) return p1;
		float mu = (isolevel - val1) / (val2 - val1);
		return p1 + mu * (p2 - p1);
	}

	glm::vec3 NormalInterp(float isolevel, glm::vec3 n1, glm::vec3 n2, float val1, float val2)
	{
		if (std::abs(val1 - val2) < 0.00001f) return n1;
		float mu = (isolevel - val1) / (val2 - val1);
		glm::vec3 n = n1 + mu * (n2 - n1);
		return glm::normalize(n);
	}

	glm::vec3 ColorInterp(float isolevel, glm::vec3 c1, glm::vec3 c2, float val1, float val2)
	{
		if (std::abs(val1 - val2) < 0.00001f) return c1;
		float mu = (isolevel - val1) / (val2 - val1);
		return c1 + mu * (c2 - c1);
	}

	void GenerateMeshFromBlocks(SparseDataBlock& sdb)
	{
		triangles.clear();
		holeEdges.clear();
		float isoLevel = 0.0f;

		for (auto& [key, block] : sdb.dataBlocks)
		{
			glm::vec3 diff = block.blockMin - sdb.gridOrigin;
			int startGx = (int)std::floor(diff.x / sdb.voxelSize + 0.5f);
			int startGy = (int)std::floor(diff.y / sdb.voxelSize + 0.5f);
			int startGz = (int)std::floor(diff.z / sdb.voxelSize + 0.5f);

			for (int z = 0; z < VpB; ++z)
			{
				for (int y = 0; y < VpB; ++y)
				{
					for (int x = 0; x < VpB; ++x)
					{
						float vals[8];
						glm::vec3 pos[8];
						glm::vec3 cols[8];
						glm::vec3 norms[8];

						float sumDist = 0.0f;
						glm::vec3 sumColor(0.0f);
						glm::vec3 sumNormal(0.0f);
						int validCnt = 0;

						const Voxel* voxels[8] = { nullptr };
						for (int i = 0; i < 8; ++i)
						{
							int gx = startGx + x + (int)cornerOffsets[i].x;
							int gy = startGy + y + (int)cornerOffsets[i].y;
							int gz = startGz + z + (int)cornerOffsets[i].z;

							pos[i] = sdb.gridOrigin + glm::vec3(gx, gy, gz) * sdb.voxelSize;

							voxels[i] = sdb.GetVoxelByIndex(gx, gy, gz);

							if (voxels[i] && voxels[i]->valid && voxels[i]->weight > 0.001f)
							{
								if (FLT_MAX == voxels[i]->signedDistance)
								{
									validCnt = 0;
									printf("Warning: Invalid SDF value encountered during mesh generation.\n");
									break;
								}

								sumDist += voxels[i]->signedDistance;
								sumColor += voxels[i]->color;
								sumNormal += voxels[i]->normal; // Sum normals
								validCnt++;
							}
						}

						if (validCnt == 0) continue;

						float avgDist = sumDist / (float)validCnt;
						glm::vec3 avgColor = sumColor / (float)validCnt;

						// Calculate average normal for gap filling
						glm::vec3 avgNormal = glm::vec3(0, 1, 0);
						if (glm::length(sumNormal) > 0.0001f)
						{
							avgNormal = glm::normalize(sumNormal);
						}

						for (int i = 0; i < 8; ++i)
						{
							if (voxels[i] && voxels[i]->valid && voxels[i]->weight > 0.001f)
							{
								vals[i] = voxels[i]->signedDistance;
								cols[i] = voxels[i]->color;
								norms[i] = voxels[i]->normal;
							}
							else
							{
								vals[i] = avgDist;
								cols[i] = avgColor;
								norms[i] = avgNormal;
							}
						}

						int idx = 0;
						if (vals[0] < isoLevel) idx |= 1;
						if (vals[1] < isoLevel) idx |= 2;
						if (vals[2] < isoLevel) idx |= 4;
						if (vals[3] < isoLevel) idx |= 8;
						if (vals[4] < isoLevel) idx |= 16;
						if (vals[5] < isoLevel) idx |= 32;
						if (vals[6] < isoLevel) idx |= 64;
						if (vals[7] < isoLevel) idx |= 128;

						if (edgeTable[idx] == 0) continue;

						glm::vec3 vList[12], cList[12], nList[12];

						auto Interp = [&](int ei, int v1, int v2)
							{
								if (edgeTable[idx] & (1 << ei))
								{
									vList[ei] = VertexInterp(isoLevel, pos[v1], pos[v2], vals[v1], vals[v2]);
									cList[ei] = ColorInterp(isoLevel, cols[v1], cols[v2], vals[v1], vals[v2]);
									nList[ei] = NormalInterp(isoLevel, norms[v1], norms[v2], vals[v1], vals[v2]);
								}
							};

						Interp(0, 0, 1); Interp(1, 1, 2); Interp(2, 2, 3); Interp(3, 3, 0);
						Interp(4, 4, 5); Interp(5, 5, 6); Interp(6, 6, 7); Interp(7, 7, 4);
						Interp(8, 0, 4); Interp(9, 1, 5); Interp(10, 2, 6); Interp(11, 3, 7);

						for (int i = 0; triTable[idx][i] != -1; i += 3)
						{
							Triangle tri;
							for (int k = 0; k < 3; ++k)
							{
								int id = triTable[idx][i + k];
								tri.v[k] = vList[id];
								tri.c[k] = cList[id];
								tri.n[k] = nList[id];
							}
							triangles.push_back(tri);
						}
					}
				}
			}
		}
	}

	void SmoothMesh(int iterations = 3)
	{
		if (triangles.empty()) return;

		float tol = 0.001f;

		std::vector<glm::vec3> uniqueVerts;
		std::vector<glm::vec3> uniqueColors;
		std::vector<glm::vec3> uniqueNormals;
		std::vector<uint32_t> indices;

		std::unordered_map<GridKey, uint32_t, GridKeyHash> vMap;

		for (const auto& t : triangles)
		{
			for (int i = 0; i < 3; ++i)
			{
				GridKey key = {
					(int)(t.v[i].x / tol),
					(int)(t.v[i].y / tol),
					(int)(t.v[i].z / tol)
				};

				if (vMap.find(key) == vMap.end())
				{
					vMap[key] = (uint32_t)uniqueVerts.size();
					uniqueVerts.push_back(t.v[i]);
					uniqueColors.push_back(t.c[i]);
					uniqueNormals.push_back(t.n[i]);
				}
				indices.push_back(vMap[key]);
			}
		}

		// Laplacian Smoothing
		std::vector<std::vector<uint32_t>> adj(uniqueVerts.size());
		for (size_t i = 0; i < indices.size(); i += 3)
		{
			uint32_t i0 = indices[i], i1 = indices[i + 1], i2 = indices[i + 2];
			adj[i0].push_back(i1); adj[i0].push_back(i2);
			adj[i1].push_back(i0); adj[i1].push_back(i2);
			adj[i2].push_back(i0); adj[i2].push_back(i1);
		}

		for (int iter = 0; iter < iterations; ++iter)
		{
			std::vector<glm::vec3> smoothed = uniqueVerts;
			for (size_t i = 0; i < uniqueVerts.size(); ++i)
			{
				if (adj[i].empty()) continue;
				glm::vec3 avg(0.0f);
				for (uint32_t n : adj[i]) avg += uniqueVerts[n];
				avg /= (float)adj[i].size();
				smoothed[i] = glm::mix(uniqueVerts[i], avg, 0.5f);
			}
			uniqueVerts = smoothed;
		}

		triangles.clear();
		for (size_t i = 0; i < indices.size(); i += 3)
		{
			Triangle tri;
			tri.v[0] = uniqueVerts[indices[i]];
			tri.v[1] = uniqueVerts[indices[i + 1]];
			tri.v[2] = uniqueVerts[indices[i + 2]];

			tri.c[0] = uniqueColors[indices[i]];
			tri.c[1] = uniqueColors[indices[i + 1]];
			tri.c[2] = uniqueColors[indices[i + 2]];

			tri.n[0] = uniqueNormals[indices[i]];
			tri.n[1] = uniqueNormals[indices[i + 1]];
			tri.n[2] = uniqueNormals[indices[i + 2]];

			triangles.push_back(tri);
		}
	}

	void SmoothBoundaries(int iterations = 10)
	{
		if (triangles.empty()) return;

		float tol = 0.001f;

		std::vector<glm::vec3> uniqueVerts;
		std::vector<glm::vec3> uniqueColors;
		std::vector<glm::vec3> uniqueNormals;
		std::vector<uint32_t> indices;

		std::unordered_map<GridKey, uint32_t, GridKeyHash> vMap;

		for (const auto& t : triangles)
		{
			for (int i = 0; i < 3; ++i)
			{
				GridKey key = {
					(int)(t.v[i].x / tol),
					(int)(t.v[i].y / tol),
					(int)(t.v[i].z / tol)
				};

				if (vMap.find(key) == vMap.end())
				{
					vMap[key] = (uint32_t)uniqueVerts.size();
					uniqueVerts.push_back(t.v[i]);
					uniqueColors.push_back(t.c[i]);
					uniqueNormals.push_back(t.n[i]);
				}
				indices.push_back(vMap[key]);
			}
		}

		std::map<std::pair<uint32_t, uint32_t>, int> edgeCount;
		std::vector<std::vector<uint32_t>> boundaryAdj(uniqueVerts.size());
		std::vector<bool> isBoundaryVert(uniqueVerts.size(), false);

		for (size_t i = 0; i < indices.size(); i += 3)
		{
			uint32_t idx[3] = { indices[i], indices[i + 1], indices[i + 2] };
			for (int j = 0; j < 3; ++j)
			{
				uint32_t a = idx[j];
				uint32_t b = idx[(j + 1) % 3];
				if (a > b) std::swap(a, b);
				edgeCount[{a, b}]++;
			}
		}

		for (auto& kv : edgeCount)
		{
			if (kv.second == 1)
			{
				uint32_t v1 = kv.first.first;
				uint32_t v2 = kv.first.second;

				isBoundaryVert[v1] = true;
				isBoundaryVert[v2] = true;

				boundaryAdj[v1].push_back(v2);
				boundaryAdj[v2].push_back(v1);
			}
		}

		for (int iter = 0; iter < iterations; ++iter)
		{
			std::vector<glm::vec3> nextVerts = uniqueVerts;

			for (size_t i = 0; i < uniqueVerts.size(); ++i)
			{
				if (!isBoundaryVert[i]) continue;
				if (boundaryAdj[i].empty()) continue;

				glm::vec3 sum(0.0f);
				for (uint32_t neighbor : boundaryAdj[i])
				{
					sum += uniqueVerts[neighbor];
				}

				glm::vec3 avg = sum / (float)boundaryAdj[i].size();
				nextVerts[i] = glm::mix(uniqueVerts[i], avg, 0.5f);
			}
			uniqueVerts = nextVerts;
		}

		triangles.clear();
		for (size_t i = 0; i < indices.size(); i += 3)
		{
			Triangle tri;
			tri.v[0] = uniqueVerts[indices[i]];
			tri.v[1] = uniqueVerts[indices[i + 1]];
			tri.v[2] = uniqueVerts[indices[i + 2]];

			tri.c[0] = uniqueColors[indices[i]];
			tri.c[1] = uniqueColors[indices[i + 1]];
			tri.c[2] = uniqueColors[indices[i + 2]];

			// Restore normals
			tri.n[0] = uniqueNormals[indices[i]];
			tri.n[1] = uniqueNormals[indices[i + 1]];
			tri.n[2] = uniqueNormals[indices[i + 2]];

			triangles.push_back(tri);
		}
	}

	void DetectHoles()
	{
		std::vector<glm::vec3> verts;
		float tol = 0.005f;

		std::unordered_map<GridKey, uint32_t, GridKeyHash> vMap;

		for (const auto& t : triangles)
		{
			for (int i = 0; i < 3; ++i)
			{
				GridKey key = {
					(int)(t.v[i].x / tol),
					(int)(t.v[i].y / tol),
					(int)(t.v[i].z / tol)
				};

				if (vMap.find(key) == vMap.end())
				{
					vMap[key] = (uint32_t)verts.size();
					verts.push_back(t.v[i]);
				}
			}
		}

		std::map<std::pair<uint32_t, uint32_t>, int> edgeCount;
		for (const auto& t : triangles)
		{
			uint32_t idx[3];
			for (int i = 0; i < 3; ++i)
			{
				GridKey key = {
					(int)(t.v[i].x / tol),
					(int)(t.v[i].y / tol),
					(int)(t.v[i].z / tol)
				};
				idx[i] = vMap[key];
			}
			for (int j = 0; j < 3; ++j)
			{
				uint32_t a = idx[j], b = idx[(j + 1) % 3];
				if (a > b) std::swap(a, b);
				edgeCount[{a, b}]++;
			}
		}

		holeEdges.clear();
		for (auto& kv : edgeCount)
		{
			if (kv.second == 1)
			{
				glm::vec3 p1 = verts[kv.first.first];
				glm::vec3 p2 = verts[kv.first.second];
				if (glm::distance(p1, p2) > 0.01f)
				{
					holeEdges.push_back({ p1, p2 });
				}
			}
		}
	}

	void Visualize(bool showMesh, bool showHoles)
	{
		struct Mesh
		{
			std::vector<glm::vec3> vertices;
			std::vector<glm::vec3> normals;
			std::vector<glm::vec3> colors;
			std::vector<uint32_t> indices;
		} mesh;

		std::unordered_map<uint64_t, uint32_t> vertexMap;

		for (const auto& tri : triangles)
		{
			uint32_t triIndices[3];
			for (int i = 0; i < 3; ++i)
			{
				auto key = Morton3D::EncodeFromVec3(tri.v[i], glm::zero<glm::vec3>(), 0.00001f);

				if (vertexMap.find(key) == vertexMap.end())
				{
					uint32_t idx = (uint32_t)mesh.vertices.size();
					vertexMap[key] = idx;
					mesh.vertices.push_back(tri.v[i]);
					mesh.normals.push_back(tri.n[i]);
					mesh.colors.push_back(tri.c[i]);
					triIndices[i] = idx;
				}
				else
				{
					triIndices[i] = vertexMap[key];
				}
				mesh.indices.push_back(triIndices[i]);
			}
		}

		if (showMesh)
		{
			auto meshEntity = Feather.CreateEntity("TSDFMesh");
			auto meshRenderable = Feather.CreateComponent<Renderable>(meshEntity);
			meshRenderable->Initialize(Renderable::GeometryMode::Triangles);

			meshRenderable->AddShader(
				Feather.CreateShader(
					"Default",
					File("../../res/Shaders/Default.vs"),
					File("../../res/Shaders/Default.fs")
				)
			);
			meshRenderable->AddShader(
				Feather.CreateShader(
					"Flat",
					File("../../res/Shaders/Flat.vs"),
					File("../../res/Shaders/Flat.fs")
				)
			);
			meshRenderable->SetActiveShaderIndex(0);

			meshRenderable->AddVertices(mesh.vertices);
			meshRenderable->AddNormals(mesh.normals);
			meshRenderable->AddColors(mesh.colors);
			meshRenderable->AddIndices(mesh.indices);

			Feather.CreateEventCallback<KeyEvent>(meshEntity, [](Entity entity, const KeyEvent& event) {
				auto renderable = Feather.GetComponent<Renderable>(entity);
				if (!renderable) return;

				if (0 == event.action)
				{
					if (GLFW_KEY_GRAVE_ACCENT == event.keyCode) renderable->NextDrawingMode();
					else if (GLFW_KEY_1 == event.keyCode) renderable->SetActiveShaderIndex(0);
					else if (GLFW_KEY_2 == event.keyCode) renderable->SetActiveShaderIndex(1);
				}
				});

			/*
			for (const auto& tri : triangles) {
				 glm::vec3 avgC = (tri.c[0] + tri.c[1] + tri.c[2]) / 3.0f;
				 VD::AddTriangle("Mesh", tri.v[0], tri.v[1], tri.v[2], glm::vec4(avgC, 1.0f));
			}
			*/
		}

		if (showHoles)
		{
			std::map<std::pair<uint32_t, uint32_t>, int> edgeCounts;

			for (size_t i = 0; i < mesh.indices.size(); i += 3)
			{
				uint32_t idx[3] = { mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2] };

				for (int j = 0; j < 3; ++j)
				{
					uint32_t a = idx[j];
					uint32_t b = idx[(j + 1) % 3];

					if (a > b) std::swap(a, b);
					edgeCounts[{a, b}]++;
				}
			}

			for (const auto& pair : edgeCounts)
			{
				if (pair.second == 1) // 공유되지 않은 엣지 발견
				{
					uint32_t idxA = pair.first.first;
					uint32_t idxB = pair.first.second;

					VD::AddLine("Holes", mesh.vertices[idxA], mesh.vertices[idxB], Color::red());
				}
			}
		}
	}
};

static inline std::string FormatWithCommas(size_t value)
{
	std::string numStr = std::to_string(value);
	int insertPosition = static_cast<int>(numStr.length()) - 3;
	while (insertPosition > 0)
	{
		numStr.insert(insertPosition, ",");
		insertPosition -= 3;
	}
	return numStr;
}

int main(int argc, char** argv)
{
	std::cout << "AppFeather" << std::endl;

	Feather.Initialize(1920, 1080);
	Feather.SetConsoleWindowIndex(3);
	Feather.SetMainWindowIndex(2);

	auto w = Feather.GetFeatherWindow();

#pragma region AppMain
	{
		auto appMain = Feather.CreateEntity("AppMain");
		Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event)
			{
				if (GLFW_KEY_ESCAPE == event.keyCode)
				{
					glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
				}
				else if (GLFW_KEY_SPACE == event.keyCode)
				{
					if (event.action == 0)
					{
						Feather.GetImmediateModeRenderSystem()->ToggleEnable();
					}
				}
			});
	}
#pragma endregion

#pragma region Camera
	{
		Entity cam = Feather.CreateEntity("Camera");
		auto pcam = Feather.CreateComponent<PerspectiveCamera>(cam);
		auto pcamMan = Feather.CreateComponent<CameraManipulatorTrackball>(cam);
		pcamMan->SetCamera(pcam);

		Feather.CreateEventCallback<FrameBufferResizeEvent>(cam, [pcam](Entity entity, const FrameBufferResizeEvent& event)
			{
				auto window = Feather.GetFeatherWindow();
				auto aspectRatio = (f32)window->GetWidth() / (f32)window->GetHeight();
				pcam->SetAspectRatio(aspectRatio);
			});

		Feather.CreateEventCallback<KeyEvent>(cam, [](Entity entity, const KeyEvent& event)
			{
				Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnKey(event);
			});

		Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity entity, const MousePositionEvent& event)
			{
				Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMousePosition(event);
			});

		Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity entity, const MouseButtonEvent& event)
			{
				auto manipulator = Feather.GetComponent<CameraManipulatorTrackball>(entity);
				manipulator->OnMouseButton(event);

				if (event.button == 0 && event.action == 0)
				{
					int fbWidth, fbHeight;
					glfwGetFramebufferSize(w->GetGLFWwindow(), &fbWidth, &fbHeight);

					int readX = static_cast<int>(event.xpos);
					int readY = fbHeight - static_cast<int>(event.ypos) - 1;

					float depth = 0.0f;
					glReadPixels(readX, readY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
					printf("Depth: %f\n", depth);
				}
			});

		Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event)
			{
				Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event);
			});
	}
#pragma endregion

	Feather.AddOnInitializeCallback([&]()
		{
			{
				PLYFormat ply;
				if (false == ply.Deserialize("D:\\Debug\\PLY\\inputA.ply"))
				{
					printf("Failed to load PLY file.\n");
					return;
				}

				size_t numberOfPoints = ply.GetPoints().size() / 3;

				std::vector<glm::vec3> points(numberOfPoints);
				std::vector<glm::vec3> normals(numberOfPoints);
				std::vector<glm::vec3> colors(numberOfPoints);

				auto [minx, miny, minz] = ply.GetAABBMin();
				auto [maxx, maxy, maxz] = ply.GetAABBMax();
				glm::vec3 aabbMin(minx, miny, minz);
				glm::vec3 aabbMax(maxx, maxy, maxz);

				{
					TS(Sorting);

					unsigned int filteredCount = 0;
					for (size_t i = 0; i < numberOfPoints; i++)
					{
						auto px = ply.GetPoints()[i * 3 + 0];
						auto py = ply.GetPoints()[i * 3 + 1];
						auto pz = ply.GetPoints()[i * 3 + 2];

						if (Configuration.filterMin.x <= px && px <= Configuration.filterMax.x &&
							Configuration.filterMin.y <= py && py <= Configuration.filterMax.y &&
							Configuration.filterMin.z <= pz && pz <= Configuration.filterMax.z)
						{
							points[filteredCount] = glm::vec3(px, py, pz);

							if (false == ply.GetNormals().empty())
							{
								auto nx = ply.GetNormals()[i * 3 + 0];
								auto ny = ply.GetNormals()[i * 3 + 1];
								auto nz = ply.GetNormals()[i * 3 + 2];
								normals[filteredCount] = glm::vec3(nx, ny, nz);
							}
							if (false == ply.GetColors().empty())
							{
								if (ply.UseAlpha())
								{
									auto cr = ply.GetColors()[i * 4 + 0];
									auto cg = ply.GetColors()[i * 4 + 1];
									auto cb = ply.GetColors()[i * 4 + 2];
									colors[filteredCount] = glm::vec3(cr, cg, cb);
								}
								else
								{
									auto cr = ply.GetColors()[i * 3 + 0];
									auto cg = ply.GetColors()[i * 3 + 1];
									auto cb = ply.GetColors()[i * 3 + 2];
									colors[filteredCount] = glm::vec3(cr, cg, cb);
								}
							}

							filteredCount++;
						}
					}
					points.resize(filteredCount);
					normals.resize(filteredCount);
					colors.resize(filteredCount);

					numberOfPoints = filteredCount;

					TE(Sorting);
				}

				std::vector<size_t> indices(numberOfPoints);
				std::iota(indices.begin(), indices.end(), 0);

				std::sort(indices.begin(), indices.end(), [&](size_t i1, size_t i2) {
					const auto& p1 = points[i1];
					const auto& p2 = points[i2];

					if (p1.x != p2.x) return p1.x < p2.x;
					if (p1.y != p2.y) return p1.y < p2.y;
					return p1.z < p2.z;
					});

				auto ReorderVector = [&](const auto& source, const std::vector<size_t>& sortedIndices) {
					using T = typename std::decay<decltype(source)>::type::value_type;
					std::vector<T> sortedData(source.size());

					for (size_t i = 0; i < source.size(); ++i) {
						sortedData[i] = source[sortedIndices[i]];
					}
					return sortedData;
					};

				if (!points.empty()) points = ReorderVector(points, indices);
				if (!normals.empty()) normals = ReorderVector(normals, indices);
				if (!colors.empty()) colors = ReorderVector(colors, indices);



				SparseDataBlock sdb;
				//sdb.FromPLY("D:\\Debug\\PLY\\inputA.ply");
				sdb.FromPoints(points.data(), normals.data(), colors.data(), numberOfPoints, aabbMin, aabbMax);

				//sdb.ShowBlocks();

				TS(MeshGeneration);

				static MeshGenerator meshGen;
				meshGen.GenerateMeshFromBlocks(sdb);
				//meshGen.SmoothMesh(3);
				meshGen.SmoothBoundaries(3);
				meshGen.DetectHoles();
				
				TE(MeshGeneration);

				meshGen.Visualize(true, true);


				alog("Total DataBlocks : %s\n", FormatWithCommas(sdb.dataBlocks.size()).c_str());
				alog("DataBlock Size : %zd\n", sizeof(DataBlock));
				alog("Total Memory : %s bytes\n", FormatWithCommas(sdb.dataBlocks.size() * sizeof(DataBlock)).c_str());
			}

#pragma region Status Panel
			{
				auto gui = Feather.GetRegistry().create();
				auto statusPanel = Feather.GetRegistry().emplace<StatusPanel>(gui);

				Feather.CreateEventCallback<MousePositionEvent>(gui, [](Entity entity, const MousePositionEvent& event)
					{
						auto& component = Feather.GetRegistry().get<StatusPanel>(entity);
						component.mouseX = event.xpos;
						component.mouseY = event.ypos;
					});
			}
#pragma endregion

		});

	Feather.Run();
	Feather.Terminate();

	return 0;
}