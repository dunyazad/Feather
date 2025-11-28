#include <libFeather.h>
#include <vector>
#include <execution>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <unordered_map>
#include <map>
#include <atomic>

// =============================================================
// Configuration
// =============================================================
struct Configuration
{
	const float voxelSize = 0.2f;

	// Filtering range
	glm::vec3 filterMin = glm::vec3(-10.0f, -10.0f, -10.0f);
	glm::vec3 filterMax = glm::vec3(10.0f, 10.0f, 10.0f);
} Configuration;

using VD = VisualDebugging;
using namespace libRxTx;

// =============================================================
// Acceleration Structure: Spatial Hashing for Points
// =============================================================
struct PointAccelerator
{
	struct Cell { std::vector<uint32_t> pointIndices; };
	std::vector<Cell> cells;
	glm::vec3 origin = glm::zero<glm::vec3>();
	glm::ivec3 dim = glm::ivec3(0);
	float cellSize = 1.0f;

	void Initialize(const std::vector<glm::vec3>& points, float range)
	{
		if (points.empty()) return;

		// 1. Calculate AABB
		glm::vec3 minP(FLT_MAX), maxP(-FLT_MAX);
		for (const auto& p : points) {
			minP = glm::min(minP, p);
			maxP = glm::max(maxP, p);
		}

		cellSize = range; // Use truncDist as cell size for efficient lookup
		minP -= glm::vec3(cellSize * 0.5f); // Padding
		maxP += glm::vec3(cellSize * 0.5f);

		origin = minP;
		glm::vec3 size = maxP - minP;
		dim = glm::ivec3(glm::ceil(size / cellSize)) + glm::ivec3(1);

		size_t totalCells = (size_t)dim.x * dim.y * dim.z;
		cells.clear();
		cells.resize(totalCells);

		// Parallel filling is tricky with vector push_back, stick to serial or pre-count
		// For speed in this specific case, serial is often fast enough if cells are coarse
		for (uint32_t i = 0; i < points.size(); ++i)
		{
			glm::ivec3 idx = glm::ivec3((points[i] - origin) / cellSize);
			if (idx.x >= 0 && idx.x < dim.x && idx.y >= 0 && idx.y < dim.y && idx.z >= 0 && idx.z < dim.z)
			{
				cells[idx.z * dim.x * dim.y + idx.y * dim.x + idx.x].pointIndices.push_back(i);
			}
		}
	}

	float GetNearest(const glm::vec3& pos, const std::vector<glm::vec3>& points, uint32_t& outIdx, float maxDist)
	{
		glm::ivec3 centerIdx = glm::ivec3((pos - origin) / cellSize);
		float minDistSq = maxDist * maxDist;
		outIdx = -1;

		// Optimize: Check bounds early
		int minX = std::max(0, centerIdx.x - 1); int maxX = std::min(dim.x - 1, centerIdx.x + 1);
		int minY = std::max(0, centerIdx.y - 1); int maxY = std::min(dim.y - 1, centerIdx.y + 1);
		int minZ = std::max(0, centerIdx.z - 1); int maxZ = std::min(dim.z - 1, centerIdx.z + 1);

		for (int z = minZ; z <= maxZ; ++z) {
			for (int y = minY; y <= maxY; ++y) {
				for (int x = minX; x <= maxX; ++x) {
					const auto& cell = cells[z * dim.x * dim.y + y * dim.x + x];
					for (uint32_t pIdx : cell.pointIndices)
					{
						glm::vec3 diff = points[pIdx] - pos;
						float d2 = glm::dot(diff, diff);

						if (d2 < minDistSq)
						{
							minDistSq = d2;
							outIdx = pIdx;
						}
					}
				}
			}
		}
		return minDistSq;
	}
};

// =============================================================
// Fast Volume: Dense Grid Implementation with Narrow Band
// =============================================================
struct FastVolume
{
	struct VoxelData {
		float sdf;
		glm::vec3 color;
		glm::vec3 normal;
		bool valid;
	};

	std::vector<VoxelData> voxels;
	std::vector<uint32_t> activeIndices; // Optimization: Only process these
	glm::ivec3 dim = glm::ivec3(0);
	glm::vec3 origin = glm::zero<glm::vec3>();
	float voxelSize = 0.2f;

	const VoxelData* GetVoxel(int x, int y, int z) const
	{
		if (x < 0 || x >= dim.x || y < 0 || y >= dim.y || z < 0 || z >= dim.z) return nullptr;
		return &voxels[(size_t)z * dim.x * dim.y + (size_t)y * dim.x + (size_t)x];
	}

	VoxelData* GetVoxelMutable(int x, int y, int z)
	{
		if (x < 0 || x >= dim.x || y < 0 || y >= dim.y || z < 0 || z >= dim.z) return nullptr;
		return &voxels[(size_t)z * dim.x * dim.y + (size_t)y * dim.x + (size_t)x];
	}

	void FromPoints(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const std::vector<glm::vec3>& colors, float vSize)
	{
		voxelSize = vSize;
		if (points.empty()) return;

		TS(Volume_Allocation);

		glm::vec3 minP(FLT_MAX), maxP(-FLT_MAX);
		for (const auto& p : points) {
			minP = glm::min(minP, p);
			maxP = glm::max(maxP, p);
		}

		float truncDist = std::max(voxelSize * 4.0f, 1.0f); // Reduced range slightly for speed
		float padding = truncDist + voxelSize * 2.0f;

		origin = glm::floor((minP - glm::vec3(padding)) / voxelSize) * voxelSize;
		glm::vec3 endP = glm::ceil((maxP + glm::vec3(padding)) / voxelSize) * voxelSize;
		glm::vec3 size = endP - origin;
		dim = glm::ivec3(glm::round(size / voxelSize));

		size_t totalVoxels = (size_t)dim.x * dim.y * dim.z;
		if (totalVoxels > 1'000'000'000) {
			printf("Error: Volume too large (%zu voxels). Increase voxelSize.\n", totalVoxels);
			return;
		}

		// Initialize with invalid state
		voxels.clear();
		voxels.resize(totalVoxels, { truncDist, glm::vec3(1.0f), glm::vec3(0,1,0), false });

		TE(Volume_Allocation);

		TS(Accelerator_Build);
		PointAccelerator accel;
		accel.Initialize(points, truncDist);
		TE(Accelerator_Build);

		TS(NarrowBand_Tagging);
		// OPTIMIZATION: Narrow Band Generation
		// Instead of iterating all voxels, we mark voxels near points.
		std::vector<uint8_t> gridFlags(totalVoxels, 0); // 0: ignore, 1: active

		int rangeCells = (int)std::ceil(truncDist / voxelSize) + 1;

		std::for_each(std::execution::par, points.begin(), points.end(), [&](const glm::vec3& p) {
			glm::vec3 localPos = (p - origin) / voxelSize;
			glm::ivec3 center = glm::ivec3(std::round(localPos.x), std::round(localPos.y), std::round(localPos.z));

			for (int z = center.z - rangeCells; z <= center.z + rangeCells; ++z) {
				if (z < 0 || z >= dim.z) continue;
				for (int y = center.y - rangeCells; y <= center.y + rangeCells; ++y) {
					if (y < 0 || y >= dim.y) continue;
					for (int x = center.x - rangeCells; x <= center.x + rangeCells; ++x) {
						if (x < 0 || x >= dim.x) continue;

						size_t idx = (size_t)z * dim.x * dim.y + (size_t)y * dim.x + x;
						// Benign data race: writing 1 to same byte is safe on x64/modern archs
						gridFlags[idx] = 1;
					}
				}
			}
			});

		activeIndices.clear();
		activeIndices.reserve(points.size() * 8); // rough estimate
		for (size_t i = 0; i < totalVoxels; ++i) {
			if (gridFlags[i]) activeIndices.push_back((uint32_t)i);
		}

		TE(NarrowBand_Tagging);

		TS(SDF_Calculation);

		// Process ONLY active voxels
		std::for_each(std::execution::par, activeIndices.begin(), activeIndices.end(), [&](uint32_t idx)
			{
				// Decode index
				int z = idx / (dim.x * dim.y);
				int rem = idx % (dim.x * dim.y);
				int y = rem / dim.x;
				int x = rem % dim.x;

				glm::vec3 vPos = origin + glm::vec3(x, y, z) * voxelSize;
				VoxelData& v = voxels[idx];

				uint32_t nearestIdx = -1;
				float distSq = accel.GetNearest(vPos, points, nearestIdx, truncDist);

				if (nearestIdx != -1)
				{
					float dist = std::sqrt(distSq);
					const glm::vec3& p = points[nearestIdx];
					const glm::vec3& n = normals.empty() ? glm::vec3(0, 1, 0) : normals[nearestIdx];

					float sdf = glm::dot(vPos - p, n);

					if (sdf < -truncDist) sdf = -truncDist;
					if (sdf > truncDist) sdf = truncDist;

					v.sdf = sdf;
					v.valid = true;

					if (!colors.empty()) v.color = colors[nearestIdx];
					if (!normals.empty()) v.normal = n;
				}
			});

		TE(SDF_Calculation);

		printf("Grid: %d x %d x %d, Active Voxels: %zu / %zu (%.2f%%)\n",
			dim.x, dim.y, dim.z, activeIndices.size(), totalVoxels,
			(float)activeIndices.size() / totalVoxels * 100.0f);
	}
};

// =============================================================
// Mesh Generator (Surface Nets with Open Surface Logic)
// =============================================================
struct Triangle { glm::vec3 v[3]; glm::vec3 n[3]; glm::vec3 c[3]; };

struct MeshGenerator
{
	std::vector<Triangle> triangles;
	std::vector<std::pair<glm::vec3, glm::vec3>> holeEdges;

	struct GridKey { int x, y, z; bool operator==(const GridKey& o) const { return x == o.x && y == o.y && z == o.z; } };
	struct GridKeyHash { size_t operator()(const GridKey& k) const { return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1); } };
	struct SNVertex { glm::vec3 pos; glm::vec3 normal; glm::vec3 color; };

	// Thread-safe container helper
	struct ThreadLocalMesh {
		std::vector<Triangle> tris;
		std::vector<std::pair<GridKey, SNVertex>> vertices;
	};

	void Generate(const FastVolume& vol)
	{
		triangles.clear();
		holeEdges.clear();
		float isoLevel = 0.0f;

		// 1. Generate Vertices (Parallel using Active List)
		// Map is slow in parallel, use vector and sort
		using KeyVertPair = std::pair<GridKey, SNVertex>;
		std::vector<KeyVertPair> allVertices;

		// Only iterate active voxels, but Surface Nets technically runs on dual cells.
		// However, active voxels cover the surface band, so it's safe to iterate them.
		// We use a concurrent vector approach to avoid locks.

		int numThreads = std::thread::hardware_concurrency();
		std::vector<std::vector<KeyVertPair>> threadVertices(numThreads);

		// Split active indices among threads or use par for_each with thread_local
		// Using chunks for simplicity
		size_t chunkSize = (vol.activeIndices.size() + numThreads - 1) / numThreads;
		std::vector<size_t> tIndices(numThreads);
		std::iota(tIndices.begin(), tIndices.end(), 0);

		const glm::ivec3 corners[8] = { {0,0,0}, {1,0,0}, {1,0,1}, {0,0,1}, {0,1,0}, {1,1,0}, {1,1,1}, {0,1,1} };
		const int edgePairs[12][2] = { {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7} };

		std::for_each(std::execution::par, tIndices.begin(), tIndices.end(), [&](size_t tID) {
			size_t start = tID * chunkSize;
			size_t end = std::min(start + chunkSize, vol.activeIndices.size());
			auto& outVerts = threadVertices[tID];

			for (size_t i = start; i < end; ++i) {
				uint32_t idx = vol.activeIndices[i];
				int z = idx / (vol.dim.x * vol.dim.y);
				int rem = idx % (vol.dim.x * vol.dim.y);
				int y = rem / vol.dim.x;
				int x = rem % vol.dim.x;

				// Boundary check for +1 neighbors
				if (x >= vol.dim.x - 1 || y >= vol.dim.y - 1 || z >= vol.dim.z - 1) continue;

				float dists[8];
				glm::vec3 colors[8];
				glm::vec3 normals[8];
				int insideCount = 0;
				bool allValid = true;

				for (int k = 0; k < 8; ++k) {
					const auto* v = vol.GetVoxel(x + corners[k].x, y + corners[k].y, z + corners[k].z);
					if (!v || !v->valid) { allValid = false; break; }

					dists[k] = v->sdf;
					colors[k] = v->color;
					normals[k] = v->normal;
					if (dists[k] < isoLevel) insideCount++;
				}

				if (!allValid || insideCount == 0 || insideCount == 8) continue;

				glm::vec3 avgPos(0.0f), avgColor(0.0f), avgNormal(0.0f);
				int intersections = 0;

				for (int e = 0; e < 12; ++e) {
					int idx1 = edgePairs[e][0]; int idx2 = edgePairs[e][1];
					if ((dists[idx1] < isoLevel) != (dists[idx2] < isoLevel)) {
						float t = (isoLevel - dists[idx1]) / (dists[idx2] - dists[idx1]);
						glm::vec3 p1 = vol.origin + glm::vec3(x + corners[idx1].x, y + corners[idx1].y, z + corners[idx1].z) * vol.voxelSize;
						glm::vec3 p2 = vol.origin + glm::vec3(x + corners[idx2].x, y + corners[idx2].y, z + corners[idx2].z) * vol.voxelSize;

						avgPos += glm::mix(p1, p2, t);
						avgColor += glm::mix(colors[idx1], colors[idx2], t);
						avgNormal += glm::mix(normals[idx1], normals[idx2], t);
						intersections++;
					}
				}

				if (intersections > 0) {
					GridKey key = { x, y, z };
					SNVertex v;
					v.pos = avgPos / (float)intersections;
					v.color = avgColor / (float)intersections;
					v.normal = glm::normalize(avgNormal);
					outVerts.push_back({ key, v });
				}
			}
			});

		// Merge vertices to map
		std::unordered_map<GridKey, SNVertex, GridKeyHash> snVertices;
		for (const auto& vec : threadVertices) {
			for (const auto& pair : vec) {
				snVertices[pair.first] = pair.second;
			}
		}

		// 2. Generate Quads
		// We can reuse activeIndices again but need to check neighbors carefully.
		// Or simply iterate the generated snVertices keys? No, keys represent cells.
		// Let's iterate snVertices directly. A vertex at (x,y,z) might form quads with neighbors.
		// Surface Nets quads are formed on edges crossing the surface.

		// To keep it simple and consistent with original logic, we iterate activeIndices again.
		// This is much faster than full grid.

		std::vector<std::vector<Triangle>> threadTriangles(numThreads);

		std::for_each(std::execution::par, tIndices.begin(), tIndices.end(), [&](size_t tID) {
			size_t start = tID * chunkSize;
			size_t end = std::min(start + chunkSize, vol.activeIndices.size());
			auto& outTris = threadTriangles[tID];

			for (size_t i = start; i < end; ++i) {
				uint32_t idx = vol.activeIndices[i];
				int z = idx / (vol.dim.x * vol.dim.y);
				int rem = idx % (vol.dim.x * vol.dim.y);
				int y = rem / vol.dim.x;
				int x = rem % vol.dim.x;

				if (x >= vol.dim.x - 1 || y >= vol.dim.y - 1 || z >= vol.dim.z - 1) continue;

				const auto* vCurr = vol.GetVoxel(x, y, z);
				if (!vCurr || !vCurr->valid) continue;
				bool bCurr = vCurr->sdf < isoLevel;

				// Helper lambda for quad addition
				auto AddQ = [&](const SNVertex& v0, const SNVertex& v1, const SNVertex& v2, const SNVertex& v3) {
					Triangle t1;
					t1.v[0] = v0.pos; t1.v[1] = v1.pos; t1.v[2] = v2.pos;
					t1.c[0] = v0.color; t1.c[1] = v1.color; t1.c[2] = v2.color;
					t1.n[0] = v0.normal; t1.n[1] = v1.normal; t1.n[2] = v2.normal;
					outTris.push_back(t1);

					Triangle t2;
					t2.v[0] = v0.pos; t2.v[1] = v2.pos; t2.v[2] = v3.pos;
					t2.c[0] = v0.color; t2.c[1] = v2.color; t2.c[2] = v3.color;
					t2.n[0] = v0.normal; t2.n[1] = v2.normal; t2.n[2] = v3.normal;
					outTris.push_back(t2);
					};

				// X Edge Check
				const auto* vX = vol.GetVoxel(x + 1, y, z);
				if (vX && vX->valid) {
					if (bCurr != (vX->sdf < isoLevel)) {
						GridKey k1{ x, y - 1, z - 1 }, k2{ x, y, z - 1 }, k3{ x, y, z }, k4{ x, y - 1, z };
						if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4)) {
							if (bCurr) AddQ(snVertices[k1], snVertices[k2], snVertices[k3], snVertices[k4]);
							else       AddQ(snVertices[k4], snVertices[k3], snVertices[k2], snVertices[k1]);
						}
					}
				}
				// Y Edge Check
				const auto* vY = vol.GetVoxel(x, y + 1, z);
				if (vY && vY->valid) {
					if (bCurr != (vY->sdf < isoLevel)) {
						GridKey k1{ x - 1, y, z - 1 }, k2{ x, y, z - 1 }, k3{ x, y, z }, k4{ x - 1, y, z };
						if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4)) {
							if (bCurr) AddQ(snVertices[k4], snVertices[k3], snVertices[k2], snVertices[k1]);
							else       AddQ(snVertices[k1], snVertices[k2], snVertices[k3], snVertices[k4]);
						}
					}
				}
				// Z Edge Check
				const auto* vZ = vol.GetVoxel(x, y, z + 1);
				if (vZ && vZ->valid) {
					if (bCurr != (vZ->sdf < isoLevel)) {
						GridKey k1{ x - 1, y - 1, z }, k2{ x, y - 1, z }, k3{ x, y, z }, k4{ x - 1, y, z };
						if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4)) {
							if (bCurr) AddQ(snVertices[k1], snVertices[k2], snVertices[k3], snVertices[k4]);
							else       AddQ(snVertices[k4], snVertices[k3], snVertices[k2], snVertices[k1]);
						}
					}
				}
			}
			});

		// Merge triangles
		for (const auto& vec : threadTriangles) {
			triangles.insert(triangles.end(), vec.begin(), vec.end());
		}
	}

	void AddQuad(const SNVertex& v0, const SNVertex& v1, const SNVertex& v2, const SNVertex& v3)
	{
		// Moved to lambda inside Generate for thread safety without locks
	}

	void SmoothMesh(int iterations)
	{
		// Existing implementation (Serial) - Optimization omitted as bottlenecks were in Grid processing
		if (triangles.empty()) return;
		float tol = 0.0001f;

		// 1. Indexing
		std::vector<glm::vec3> verts, cols, norms;
		std::vector<uint32_t> indices;
		std::unordered_map<GridKey, uint32_t, GridKeyHash> vMap;

		for (const auto& t : triangles) {
			for (int i = 0; i < 3; ++i) {
				GridKey key = { (int)(t.v[i].x / tol), (int)(t.v[i].y / tol), (int)(t.v[i].z / tol) };
				if (vMap.find(key) == vMap.end()) {
					vMap[key] = (uint32_t)verts.size();
					verts.push_back(t.v[i]);
					cols.push_back(t.c[i]);
					norms.push_back(t.n[i]);
				}
				indices.push_back(vMap[key]);
			}
		}

		// 2. Adjacency
		std::vector<std::vector<uint32_t>> adj(verts.size());
		for (size_t i = 0; i < indices.size(); i += 3) {
			uint32_t i0 = indices[i], i1 = indices[i + 1], i2 = indices[i + 2];
			adj[i0].push_back(i1); adj[i0].push_back(i2);
			adj[i1].push_back(i0); adj[i1].push_back(i2);
			adj[i2].push_back(i0); adj[i2].push_back(i1);
		}

		// 3. Smoothing
		for (int iter = 0; iter < iterations; ++iter) {
			std::vector<glm::vec3> nextVerts = verts;
			std::for_each(std::execution::par, nextVerts.begin(), nextVerts.end(), [&](glm::vec3& nv) {
				// Note: accessing adj by index is safe, accessing verts is safe (read-only)
				// Need actual index...
				// For brevity, using serial loop logic inside parallel structure is tricky without index.
				// Kept basic:
				});

			// Simple serial loop for safety in this snippet context
			for (size_t i = 0; i < verts.size(); ++i) {
				if (adj[i].empty()) continue;
				glm::vec3 sum(0.0f);
				for (uint32_t n : adj[i]) sum += verts[n];
				nextVerts[i] = glm::mix(verts[i], sum / (float)adj[i].size(), 0.5f);
			}
			verts = nextVerts;
		}

		// 4. Rebuild
		triangles.clear();
		for (size_t i = 0; i < indices.size(); i += 3) {
			Triangle tri;
			for (int k = 0; k < 3; ++k) {
				tri.v[k] = verts[indices[i + k]];
				tri.c[k] = cols[indices[i + k]];
				tri.n[k] = norms[indices[i + k]];
			}
			triangles.push_back(tri);
		}
	}

	void DetectHoles()
	{
		float tol = 0.0001f;
		std::map<std::pair<int, int>, int> edges;

		std::unordered_map<GridKey, int, GridKeyHash> vMap;
		int vCount = 0;
		std::vector<int> triIndices;
		std::vector<glm::vec3> tempVerts;

		for (const auto& t : triangles) {
			for (int i = 0; i < 3; ++i) {
				GridKey key = { (int)(t.v[i].x / tol), (int)(t.v[i].y / tol), (int)(t.v[i].z / tol) };
				if (vMap.find(key) == vMap.end()) {
					vMap[key] = vCount++;
					tempVerts.push_back(t.v[i]);
				}
				triIndices.push_back(vMap[key]);
			}
		}

		for (size_t i = 0; i < triIndices.size(); i += 3) {
			int idx[3] = { triIndices[i], triIndices[i + 1], triIndices[i + 2] };
			for (int k = 0; k < 3; ++k) {
				int a = idx[k]; int b = idx[(k + 1) % 3];
				if (a > b) std::swap(a, b);
				edges[{a, b}]++;
			}
		}

		holeEdges.clear();
		for (auto& kv : edges) {
			if (kv.second == 1) {
				holeEdges.push_back({ tempVerts[kv.first.first], tempVerts[kv.first.second] });
			}
		}
	}

	void Visualize(bool showMesh, bool showHoles)
	{
		if (showMesh)
		{
			std::vector<glm::vec3> v, n, c;
			std::vector<uint32_t> ind;
			uint32_t idx = 0;
			// Reserve to avoid reallocations
			v.reserve(triangles.size() * 3);
			n.reserve(triangles.size() * 3);
			c.reserve(triangles.size() * 3);
			ind.reserve(triangles.size() * 3);

			for (const auto& t : triangles) {
				v.push_back(t.v[0]); v.push_back(t.v[1]); v.push_back(t.v[2]);
				n.push_back(t.n[0]); n.push_back(t.n[1]); n.push_back(t.n[2]);
				c.push_back(t.c[0]); c.push_back(t.c[1]); c.push_back(t.c[2]);
				ind.push_back(idx++); ind.push_back(idx++); ind.push_back(idx++);
			}

			auto meshEntity = Feather.CreateEntity("TSDFMesh");
			auto meshRenderable = Feather.CreateComponent<Renderable>(meshEntity);
			meshRenderable->Initialize(Renderable::GeometryMode::Triangles);

			meshRenderable->AddShader(Feather.CreateShader("Default", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
			meshRenderable->AddShader(Feather.CreateShader("TwoSide", File("../../res/Shaders/TwoSide.vs"), File("../../res/Shaders/TwoSide.fs")));
			meshRenderable->SetActiveShaderIndex(0);

			meshRenderable->AddVertices(v);
			meshRenderable->AddNormals(n);
			meshRenderable->AddColors(c);
			meshRenderable->AddIndices(ind);

			Feather.CreateEventCallback<KeyEvent>(meshEntity, [](Entity entity, const KeyEvent& event) {
				auto renderable = Feather.GetComponent<Renderable>(entity);
				if (!renderable) return;
				if (0 == event.action) {
					if (GLFW_KEY_GRAVE_ACCENT == event.keyCode) renderable->NextDrawingMode();
					else if (GLFW_KEY_1 == event.keyCode) renderable->SetActiveShaderIndex(0);
					else if (GLFW_KEY_2 == event.keyCode) renderable->SetActiveShaderIndex(1);
				}
				});
		}

		if (showHoles) {
			for (const auto& edge : holeEdges) {
				VD::AddLine("Holes", edge.first, edge.second, Color::red());
			}
		}
	}

	void ExportPLY(const std::string& filename)
	{
		PLYFormat ply;

		std::unordered_map<GridKey, uint32_t, GridKeyHash> vMap;
		float tol = 0.0001f;

		for (const auto& t : triangles)
		{
			uint32_t faceIndices[3];

			for (int i = 0; i < 3; ++i)
			{
				GridKey key = {
					(int)(t.v[i].x / tol),
					(int)(t.v[i].y / tol),
					(int)(t.v[i].z / tol)
				};

				if (vMap.find(key) == vMap.end())
				{
					uint32_t newIdx = (uint32_t)vMap.size();
					vMap[key] = newIdx;

					ply.AddPoint(t.v[i].x, t.v[i].y, t.v[i].z);
					ply.AddNormal(t.n[i].x, t.n[i].y, t.n[i].z);
					ply.AddColor(t.c[i].x, t.c[i].y, t.c[i].z);

					faceIndices[i] = newIdx;
				}
				else
				{
					faceIndices[i] = vMap[key];
				}
			}

			ply.AddFace(faceIndices[0], faceIndices[1], faceIndices[2]);
		}

		if (ply.Serialize(filename))
		{
			printf("Exported PLY: %s (Verts: %zu, Tris: %zu)\n", filename.c_str(), vMap.size(), triangles.size());
		}
		else
		{
			printf("Failed to export PLY: %s\n", filename.c_str());
		}
	}
};

static inline std::string FormatWithCommas(size_t value) {
	std::string numStr = std::to_string(value);
	int insertPosition = static_cast<int>(numStr.length()) - 3;
	while (insertPosition > 0) { numStr.insert(insertPosition, ","); insertPosition -= 3; }
	return numStr;
}

// =============================================================
// MAIN
// =============================================================
int main(int argc, char** argv)
{
	std::cout << "AppFeather High-Perf Surface Reconstruction" << std::endl;

	Feather.Initialize(1920, 1080);
	Feather.SetConsoleWindowIndex(3);
	Feather.SetMainWindowIndex(2);

	{
		auto appMain = Feather.CreateEntity("AppMain");
		Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
			if (GLFW_KEY_ESCAPE == event.keyCode) glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
			else if (GLFW_KEY_SPACE == event.keyCode && event.action == 0) Feather.GetImmediateModeRenderSystem()->ToggleEnable();
			});
	}

	{
		Entity cam = Feather.CreateEntity("Camera");
		auto pcam = Feather.CreateComponent<PerspectiveCamera>(cam);
		auto pcamMan = Feather.CreateComponent<CameraManipulatorTrackball>(cam);
		pcamMan->SetCamera(pcam);

		Feather.CreateEventCallback<FrameBufferResizeEvent>(cam, [pcam](Entity entity, const FrameBufferResizeEvent& event) {
			auto window = Feather.GetFeatherWindow();
			pcam->SetAspectRatio((f32)window->GetWidth() / (f32)window->GetHeight());
			});
		Feather.CreateEventCallback<KeyEvent>(cam, [](Entity e, const KeyEvent& ev) { Feather.GetComponent<CameraManipulatorTrackball>(e)->OnKey(ev); });
		Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity e, const MousePositionEvent& ev) { Feather.GetComponent<CameraManipulatorTrackball>(e)->OnMousePosition(ev); });
		Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity e, const MouseButtonEvent& ev) { Feather.GetComponent<CameraManipulatorTrackball>(e)->OnMouseButton(ev); });
		Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity e, const MouseWheelEvent& ev) { Feather.GetRegistry().get<CameraManipulatorTrackball>(e).OnMouseWheel(ev); });
	}

	Feather.AddOnInitializeCallback([&]()
		{
			{
				PLYFormat ply;
				if (false == ply.Deserialize("D:\\Debug\\PLY\\inputA.ply")) {
					printf("Failed to load PLY file.\n");
					return;
				}
				printf("PLY Loaded: %zu points\n", ply.GetPoints().size() / 3);

				TS(Filtering);

				size_t rawCount = ply.GetPoints().size() / 3;
				std::vector<glm::vec3> points, normals, colors;
				points.reserve(rawCount);
				normals.reserve(rawCount);
				colors.reserve(rawCount);

				bool hasN = !ply.GetNormals().empty();
				bool hasC = !ply.GetColors().empty();
				bool useAlpha = ply.UseAlpha();

				for (size_t i = 0; i < rawCount; ++i)
				{
					float px = ply.GetPoints()[i * 3];
					float py = ply.GetPoints()[i * 3 + 1];
					float pz = ply.GetPoints()[i * 3 + 2];

					if (Configuration.filterMin.x <= px && px <= Configuration.filterMax.x &&
						Configuration.filterMin.y <= py && py <= Configuration.filterMax.y &&
						Configuration.filterMin.z <= pz && pz <= Configuration.filterMax.z)
					{
						points.emplace_back(px, py, pz);

						if (hasN) {
							normals.emplace_back(ply.GetNormals()[i * 3], ply.GetNormals()[i * 3 + 1], ply.GetNormals()[i * 3 + 2]);
						}
						if (hasC) {
							int stride = useAlpha ? 4 : 3;
							colors.emplace_back(ply.GetColors()[i * stride], ply.GetColors()[i * stride + 1], ply.GetColors()[i * stride + 2]);
						}
					}
				}

				points.shrink_to_fit();
				normals.shrink_to_fit();
				colors.shrink_to_fit();

				printf("Filtered: %zu points (from %zu)\n", points.size(), rawCount);
				TE(Filtering);

				if (points.empty()) {
					printf("No points remained after filtering!\n");
					return;
				}

				TS(TOTAL);

				TS(Sorting);
				std::vector<size_t> indices(points.size());
				std::iota(indices.begin(), indices.end(), 0);
				std::sort(std::execution::par, indices.begin(), indices.end(), [&](size_t i1, size_t i2) {
					return points[i1].z < points[i2].z;
					});

				auto Reorder = [&](auto& vec, const auto& idxs) {
					if (vec.empty()) return;
					auto temp = vec;
					for (size_t i = 0; i < vec.size(); ++i) vec[i] = temp[idxs[i]];
					};
				Reorder(points, indices);
				if (!normals.empty()) Reorder(normals, indices);
				if (!colors.empty()) Reorder(colors, indices);
				TE(Sorting);

				static FastVolume volume;
				volume.FromPoints(points, normals, colors, Configuration.voxelSize);

				TS(Mesh_Total);
				static MeshGenerator meshGen;
				meshGen.Generate(volume);

				// 스무딩 & 구멍 감지 (필요시 주석 해제)
				//meshGen.SmoothMesh(5);
				meshGen.DetectHoles();
				TE(Mesh_Total);

				TE(TOTAL);

				meshGen.Visualize(true, true);

				meshGen.ExportPLY("D:\\Debug\\PLY\\output_mesh.ply");
			}

			{
				auto gui = Feather.GetRegistry().create();
				auto statusPanel = Feather.GetRegistry().emplace<StatusPanel>(gui);
				Feather.CreateEventCallback<MousePositionEvent>(gui, [](Entity entity, const MousePositionEvent& event) {
					auto& component = Feather.GetRegistry().get<StatusPanel>(entity);
					component.mouseX = event.xpos; component.mouseY = event.ypos;
					});
			}
		});

	Feather.Run();
	Feather.Terminate();

	return 0;
}
