#include <libFeather.h>
#include <unordered_map>
#include <map>
#include <numeric>
#include <algorithm>
#include <execution>
#include <mutex>
#include <shared_mutex>
#include <unordered_set>
#include <memory>

struct Configuration
{
    const float voxelSize = 0.2f;
    const int sdfOffset = 2; // Surface Nets need slightly wider search to find neighbors
    //glm::vec3 filterMin = glm::vec3(-20.0f, -20.0f, -20.0f);
    //glm::vec3 filterMax = glm::vec3(20.0f, 20.0f, 20.0f);
    glm::vec3 filterMin = glm::vec3(-10.0f, -10.0f, -10.0f);
    glm::vec3 filterMax = glm::vec3(10.0f, 10.0f, 10.0f);
} Configuration;

using VD = VisualDebugging;
using namespace libRxTx;

const int VpB = 8;
const int VpBHalf = 4;

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
    std::mutex blockMutex; // Mutex for thread-safe voxel updates

    void Initialize()
    {
        memset(voxels, 0, sizeof(Voxel) * VpB * VpB * VpB);
    }
};

typedef uint64_t DataBlockKey;

struct SparseDataBlock
{
    float voxelSize = Configuration.voxelSize;
    glm::vec3 gridOrigin = glm::zero<glm::vec3>();

    // Changed to unique_ptr to prevent object slicing and allow pointer stability
    std::unordered_map<DataBlockKey, std::unique_ptr<DataBlock>> dataBlocks;

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

        return &it->second->voxels[lz * VpB * VpB + ly * VpB + lx];
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
        int searchRange = Configuration.sdfOffset;

        // Prepare indices for parallel execution
        std::vector<size_t> indices(numberOfPoints);
        std::iota(indices.begin(), indices.end(), 0);

        // ------------------------------------------------------------------
        // Pass 1: Collect required DataBlocks and Allocate them
        // ------------------------------------------------------------------
        {
            TS(Pass1_Alloc);
            // [Fix]: Use Map instead of Set to store blockMin along with the Key
            // This avoids the need for Morton3D::DecodeToAABB
            std::unordered_map<DataBlockKey, glm::vec3> blocksToAllocate;
            std::mutex mapMutex;

            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
                {
                    auto p = points[i];
                    glm::vec3 vecFromOrigin = p - gridOrigin;
                    int centerGx = (int)std::floor(vecFromOrigin.x / voxelSize);
                    int centerGy = (int)std::floor(vecFromOrigin.y / voxelSize);
                    int centerGz = (int)std::floor(vecFromOrigin.z / voxelSize);

                    // We scan broadly to capture all blocks that might be touched
                    for (int dz = -searchRange; dz <= searchRange; dz += searchRange)
                    {
                        for (int dy = -searchRange; dy <= searchRange; dy += searchRange)
                        {
                            for (int dx = -searchRange; dx <= searchRange; dx += searchRange)
                            {
                                int gx = centerGx + dx;
                                int gy = centerGy + dy;
                                int gz = centerGz + dz;

                                int bx = (int)std::floor((float)gx / VpB);
                                int by = (int)std::floor((float)gy / VpB);
                                int bz = (int)std::floor((float)gz / VpB);

                                float blockSizeCurr = voxelSize * VpB;
                                glm::vec3 blockMin = gridOrigin + glm::vec3(bx * blockSizeCurr, by * blockSizeCurr, bz * blockSizeCurr);
                                auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSizeCurr);

                                {
                                    std::lock_guard<std::mutex> lock(mapMutex);
                                    // Store blockMin so we don't have to decode it later
                                    blocksToAllocate[key] = blockMin;
                                }
                            }
                        }
                    }
                });

            // Allocate blocks in main thread (safe map modification)
            for (const auto& kv : blocksToAllocate)
            {
                DataBlockKey key = kv.first;
                glm::vec3 bMin = kv.second;

                if (dataBlocks.find(key) == dataBlocks.end())
                {
                    dataBlocks[key] = std::make_unique<DataBlock>();
                    dataBlocks[key]->Initialize();
                    dataBlocks[key]->blockMin = bMin; // [Fix]: Assign stored blockMin directly
                }
            }
            TE(Pass1_Alloc);
        }

        // ------------------------------------------------------------------
        // Pass 2: Update Voxels in Parallel (Read-only Map Access)
        // ------------------------------------------------------------------
        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
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

                // Thread-local cache for the last accessed block to reduce map lookups
                DataBlockKey lastKey = (DataBlockKey)-1;
                DataBlock* cachedBlock = nullptr;

                for (int dz = -searchRange; dz <= searchRange; ++dz)
                {
                    for (int dy = -searchRange; dy <= searchRange; ++dy)
                    {
                        for (int dx = -searchRange; dx <= searchRange; ++dx)
                        {
                            int gx = centerGx + dx;
                            int gy = centerGy + dy;
                            int gz = centerGz + dz;

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

                            DataBlock* targetBlock = nullptr;

                            if (key == lastKey && cachedBlock)
                            {
                                targetBlock = cachedBlock;
                            }
                            else
                            {
                                auto it = dataBlocks.find(key);
                                if (it != dataBlocks.end())
                                {
                                    targetBlock = it->second.get();
                                    lastKey = key;
                                    cachedBlock = targetBlock;
                                }
                            }

                            if (targetBlock)
                            {
                                int lx = gx % VpB;
                                if (lx < 0) lx += VpB;
                                int ly = gy % VpB;
                                if (ly < 0) ly += VpB;
                                int lz = gz % VpB;
                                if (lz < 0) lz += VpB;

                                // Critical Section: Voxel Update
                                // We use a fine-grained lock per block to minimize contention
                                std::lock_guard<std::mutex> lock(targetBlock->blockMutex);

                                Voxel& voxel = targetBlock->voxels[lz * VpB * VpB + ly * VpB + lx];

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
                                    voxel.weight = newW;
                                }
                            }
                        }
                    }
                }
            });
        TE(Occupy);
    }
};

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

    struct GridKey { int x, y, z; bool operator==(const GridKey& o) const { return x == o.x && y == o.y && z == o.z; } };
    struct GridKeyHash { size_t operator()(const GridKey& k) const { return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1); } };
    struct SNVertex { glm::vec3 pos; glm::vec3 normal; glm::vec3 color; };

    void Generate(SparseDataBlock& sdb)
    {
        triangles.clear();
        holeEdges.clear();
        float isoLevel = 0.0f;

        std::unordered_map<GridKey, SNVertex, GridKeyHash> snVertices;

        const glm::ivec3 corners[8] = { {0,0,0}, {1,0,0}, {1,0,1}, {0,0,1}, {0,1,0}, {1,1,0}, {1,1,1}, {0,1,1} };
        const int edgePairs[12][2] = { {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7} };

        // Pass 1: Generate Surface Nets Vertices
        // Modified loop to handle unique_ptr
        for (auto& [key, blockPtr] : sdb.dataBlocks)
        {
            DataBlock& block = *blockPtr; // Dereference unique_ptr
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
                        int gx = startGx + x;
                        int gy = startGy + y;
                        int gz = startGz + z;

                        // 8 corners check
                        float dists[8];
                        glm::vec3 colors[8];
                        glm::vec3 normals[8];
                        int insideCount = 0;
                        bool allValid = true;

                        for (int i = 0; i < 8; ++i)
                        {
                            const auto* v = sdb.GetVoxelByIndex(gx + corners[i].x, gy + corners[i].y, gz + corners[i].z);
                            if (!v || !v->valid) { allValid = false; break; }

                            dists[i] = v->signedDistance;
                            colors[i] = v->color;
                            normals[i] = v->normal;
                            if (dists[i] < isoLevel) insideCount++;
                        }

                        if (!allValid || insideCount == 0 || insideCount == 8) continue;

                        glm::vec3 avgPos(0.0f), avgColor(0.0f), avgNormal(0.0f);
                        int intersections = 0;

                        for (int e = 0; e < 12; ++e)
                        {
                            int idx1 = edgePairs[e][0]; int idx2 = edgePairs[e][1];
                            if ((dists[idx1] < isoLevel) != (dists[idx2] < isoLevel))
                            {
                                float t = (isoLevel - dists[idx1]) / (dists[idx2] - dists[idx1]);
                                glm::vec3 p1 = sdb.gridOrigin + glm::vec3(gx + corners[idx1].x, gy + corners[idx1].y, gz + corners[idx1].z) * sdb.voxelSize;
                                glm::vec3 p2 = sdb.gridOrigin + glm::vec3(gx + corners[idx2].x, gy + corners[idx2].y, gz + corners[idx2].z) * sdb.voxelSize;

                                avgPos += glm::mix(p1, p2, t);
                                avgColor += glm::mix(colors[idx1], colors[idx2], t);
                                avgNormal += glm::mix(normals[idx1], normals[idx2], t);
                                intersections++;
                            }
                        }

                        if (intersections > 0)
                        {
                            GridKey key = { gx, gy, gz };
                            SNVertex v;
                            v.pos = avgPos / (float)intersections;
                            v.color = avgColor / (float)intersections;
                            v.normal = glm::normalize(avgNormal);
                            snVertices[key] = v;
                        }
                    }
                }
            }
        }

        // Pass 2: Generate Quads
        // Modified loop to handle unique_ptr
        for (auto& [key, blockPtr] : sdb.dataBlocks)
        {
            DataBlock& block = *blockPtr; // Dereference unique_ptr
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
                        int gx = startGx + x;
                        int gy = startGy + y;
                        int gz = startGz + z;

                        const auto* vCurr = sdb.GetVoxelByIndex(gx, gy, gz);
                        if (!vCurr || !vCurr->valid) continue;
                        bool bCurr = vCurr->signedDistance < isoLevel;

                        // X Edge Check
                        const auto* vX = sdb.GetVoxelByIndex(gx + 1, gy, gz);
                        if (vX && vX->valid)
                        {
                            if (bCurr != (vX->signedDistance < isoLevel))
                            {
                                GridKey k1{ gx, gy - 1, gz - 1 }, k2{ gx, gy, gz - 1 }, k3{ gx, gy, gz }, k4{ gx, gy - 1, gz };
                                if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4))
                                {
                                    if (bCurr) AddQuad(snVertices[k1], snVertices[k2], snVertices[k3], snVertices[k4]);
                                    else       AddQuad(snVertices[k4], snVertices[k3], snVertices[k2], snVertices[k1]);
                                }
                            }
                        }
                        // Y Edge Check
                        const auto* vY = sdb.GetVoxelByIndex(gx, gy + 1, gz);
                        if (vY && vY->valid)
                        {
                            if (bCurr != (vY->signedDistance < isoLevel))
                            {
                                GridKey k1{ gx - 1, gy, gz - 1 }, k2{ gx, gy, gz - 1 }, k3{ gx, gy, gz }, k4{ gx - 1, gy, gz };
                                if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4))
                                {
                                    if (bCurr) AddQuad(snVertices[k4], snVertices[k3], snVertices[k2], snVertices[k1]);
                                    else       AddQuad(snVertices[k1], snVertices[k2], snVertices[k3], snVertices[k4]);
                                }
                            }
                        }
                        // Z Edge Check
                        const auto* vZ = sdb.GetVoxelByIndex(gx, gy, gz + 1);
                        if (vZ && vZ->valid)
                        {
                            if (bCurr != (vZ->signedDistance < isoLevel))
                            {
                                GridKey k1{ gx - 1, gy - 1, gz }, k2{ gx, gy - 1, gz }, k3{ gx, gy, gz }, k4{ gx - 1, gy, gz };
                                if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4))
                                {
                                    if (bCurr) AddQuad(snVertices[k1], snVertices[k2], snVertices[k3], snVertices[k4]);
                                    else       AddQuad(snVertices[k4], snVertices[k3], snVertices[k2], snVertices[k1]);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    void AddQuad(const SNVertex& v0, const SNVertex& v1, const SNVertex& v2, const SNVertex& v3)
    {
        Triangle t1;
        t1.v[0] = v0.pos; t1.v[1] = v1.pos; t1.v[2] = v2.pos;
        t1.c[0] = v0.color; t1.c[1] = v1.color; t1.c[2] = v2.color;
        t1.n[0] = v0.normal; t1.n[1] = v1.normal; t1.n[2] = v2.normal;
        triangles.push_back(t1);

        Triangle t2;
        t2.v[0] = v0.pos; t2.v[1] = v2.pos; t2.v[2] = v3.pos;
        t2.c[0] = v0.color; t2.c[1] = v2.color; t2.c[2] = v3.color;
        t2.n[0] = v0.normal; t2.n[1] = v2.normal; t2.n[2] = v3.normal;
        triangles.push_back(t2);
    }

    void SmoothMesh(int iterations)
    {
        if (triangles.empty()) return;
        float tol = 0.0001f;

        // 1. Indexing (Weld Vertices)
        std::vector<glm::vec3> verts, cols, norms;
        std::vector<uint32_t> indices;
        std::unordered_map<GridKey, uint32_t, GridKeyHash> vMap;

        for (const auto& t : triangles)
        {
            for (int i = 0; i < 3; ++i)
            {
                GridKey key = { (int)(t.v[i].x / tol), (int)(t.v[i].y / tol), (int)(t.v[i].z / tol) };
                if (vMap.find(key) == vMap.end())
                {
                    vMap[key] = (uint32_t)verts.size();
                    verts.push_back(t.v[i]);
                    cols.push_back(t.c[i]);
                    norms.push_back(t.n[i]);
                }
                indices.push_back(vMap[key]);
            }
        }

        // 2. Build Adjacency
        std::vector<std::vector<uint32_t>> adj(verts.size());
        for (size_t i = 0; i < indices.size(); i += 3)
        {
            uint32_t i0 = indices[i], i1 = indices[i + 1], i2 = indices[i + 2];
            adj[i0].push_back(i1); adj[i0].push_back(i2);
            adj[i1].push_back(i0); adj[i1].push_back(i2);
            adj[i2].push_back(i0); adj[i2].push_back(i1);
        }

        // 3. Laplacian Smoothing
        for (int iter = 0; iter < iterations; ++iter)
        {
            std::vector<glm::vec3> nextVerts = verts;
            for (size_t i = 0; i < verts.size(); ++i)
            {
                if (adj[i].empty()) continue;
                glm::vec3 sum(0.0f);
                for (uint32_t n : adj[i]) sum += verts[n];
                nextVerts[i] = glm::mix(verts[i], sum / (float)adj[i].size(), 0.5f);
            }
            verts = nextVerts;
        }

        // 4. Rebuild Triangles
        triangles.clear();
        for (size_t i = 0; i < indices.size(); i += 3)
        {
            Triangle tri;
            for (int k = 0; k < 3; ++k)
            {
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

        for (const auto& t : triangles)
        {
            for (int i = 0; i < 3; ++i)
            {
                GridKey key = { (int)(t.v[i].x / tol), (int)(t.v[i].y / tol), (int)(t.v[i].z / tol) };
                if (vMap.find(key) == vMap.end())
                {
                    vMap[key] = vCount++;
                    tempVerts.push_back(t.v[i]);
                }
                triIndices.push_back(vMap[key]);
            }
        }

        for (size_t i = 0; i < triIndices.size(); i += 3)
        {
            int idx[3] = { triIndices[i], triIndices[i + 1], triIndices[i + 2] };
            for (int k = 0; k < 3; ++k)
            {
                int a = idx[k]; int b = idx[(k + 1) % 3];
                if (a > b) std::swap(a, b);
                edges[{a, b}]++;
            }
        }

        holeEdges.clear();
        for (auto& kv : edges)
        {
            if (kv.second == 1)
            {
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
            for (const auto& t : triangles)
            {
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
                if (0 == event.action)
                {
                    if (GLFW_KEY_GRAVE_ACCENT == event.keyCode) renderable->NextDrawingMode();
                    else if (GLFW_KEY_1 == event.keyCode) renderable->SetActiveShaderIndex(0);
                    else if (GLFW_KEY_2 == event.keyCode) renderable->SetActiveShaderIndex(1);
                }
                });
        }

        if (showHoles)
        {
            for (const auto& edge : holeEdges)
            {
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

                TS(TOTAL);

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
                sdb.FromPoints(points.data(), normals.data(), colors.data(), numberOfPoints, aabbMin, aabbMax);

                TS(MeshGeneration);

                static MeshGenerator meshGen;
                meshGen.Generate(sdb);
                //meshGen.SmoothMesh(3);
                meshGen.DetectHoles();

                TE(MeshGeneration);

                TE(TOTAL);

                meshGen.Visualize(true, true);

                meshGen.ExportPLY("D:\\Debug\\PLY\\output.ply");

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
