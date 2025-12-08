#include <libFeather.h>
#include <vector>
#include <algorithm>
#include <execution>
#include <mutex>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <unordered_map>

// [Helper] FormatWithCommas function definition
static inline std::string FormatWithCommas(size_t value) {
    std::string numStr = std::to_string(value);
    int insertPosition = static_cast<int>(numStr.length()) - 3;
    while (insertPosition > 0) { numStr.insert(insertPosition, ","); insertPosition -= 3; }
    return numStr;
}

// [Config] Configuration struct definition
struct Configuration {
    const float voxelSize = 0.1f;
    const int sdfOffset = 1;
    glm::vec3 filterMin = glm::vec3(-10.0f, -10.0f, -10.0f);
    glm::vec3 filterMax = glm::vec3(10.0f, 10.0f, 10.0f);
    //glm::vec3 filterMin = glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    //glm::vec3 filterMax = glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX);
} Configuration;

using VD = VisualDebugging;
using namespace libRxTx;

const int VpB = 8;

// Voxel and DataBlock struct definition
struct Voxel {
    float signedDistance;
    float weight;
    glm::vec3 normal;
    glm::vec3 color;
    uint32_t valid;
};

struct DataBlock {
    glm::vec3 blockMin;
    Voxel voxels[VpB * VpB * VpB];
    std::mutex blockMutex;

    DataBlock() = default;
    DataBlock(const DataBlock& o) { blockMin = o.blockMin; std::memcpy(voxels, o.voxels, sizeof(voxels)); }
    void Initialize() { std::memset(voxels, 0, sizeof(voxels)); }
};

// 1. Direct Access Grid struct definition
struct DirectAccessGrid {
    std::vector<DataBlock> pool;
    std::vector<DataBlock*> lookup;
    glm::ivec3 dim;
    glm::vec3 origin;
    float blockSize;
    int totalCells;

    void Initialize(const glm::vec3& min, const glm::vec3& max) {
        blockSize = Configuration.voxelSize * VpB;
        origin = min - glm::vec3(blockSize);
        glm::vec3 size = (max - min) + glm::vec3(blockSize * 2.0f);
        dim.x = (int)std::ceil(size.x / blockSize);
        dim.y = (int)std::ceil(size.y / blockSize);
        dim.z = (int)std::ceil(size.z / blockSize);
        totalCells = dim.x * dim.y * dim.z;
        lookup.assign(totalCells, nullptr);
        pool.clear();
    }

    inline int GetIndex(int bx, int by, int bz) const {
        if (bx < 0 || bx >= dim.x || by < 0 || by >= dim.y || bz < 0 || bz >= dim.z) return -1;
        return bx + by * dim.x + bz * dim.x * dim.y;
    }
    inline int GetIndexFromPos(const glm::vec3& p) const {
        glm::vec3 d = p - origin;
        return GetIndex((int)(d.x / blockSize), (int)(d.y / blockSize), (int)(d.z / blockSize));
    }
};

// MeshGenerator struct definition
struct MeshGenerator
{
    struct Triangle { glm::vec3 v[3]; glm::vec3 n[3]; glm::vec3 c[3]; };
    struct SNVertex { glm::vec3 pos; glm::vec3 normal; glm::vec3 color; };

    struct GridKey { int x, y, z; bool operator==(const GridKey& o) const { return x == o.x && y == o.y && z == o.z; } };
    struct GridKeyHash { size_t operator()(const GridKey& k) const { return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1); } };

    // [Members]
    std::vector<Triangle> triangles;
    std::vector<std::pair<glm::vec3, glm::vec3>> holeEdges;

    // Cache Data: Map is fastest here (sparse data)
    std::unordered_map<GridKey, SNVertex, GridKeyHash> snVertices;
    std::mutex vertMutex, triMutex;

    void Generate(DirectAccessGrid& grid)
    {
        triangles.clear();
        snVertices.clear();
        holeEdges.clear();

        float isoLevel = 0.0f;
        float voxelSize = Configuration.voxelSize;

        // Reservation (from previous success metrics)
        size_t estTris = grid.pool.size() * 96;
        triangles.reserve(estTris);
        snVertices.reserve(estTris * 0.6f);

        const glm::ivec3 corners[8] = { {0,0,0},{1,0,0},{1,0,1},{0,0,1},{0,1,0},{1,1,0},{1,1,1},{0,1,1} };
        const int edgePairs[12][2] = { {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7} };

        // 1. Pass 1: Vertices (Parallel)
        // Process per block in parallel, collect results in local buffer, then insert to Map -> Minimize Lock Contention
        std::for_each(std::execution::par, grid.pool.begin(), grid.pool.end(), [&](DataBlock& block) {
            std::vector<std::pair<GridKey, SNVertex>> localVerts; localVerts.reserve(64);
            glm::vec3 diff = block.blockMin - grid.origin;
            int startBx = (int)(diff.x / grid.blockSize + 0.5f);
            int startBy = (int)(diff.y / grid.blockSize + 0.5f);
            int startBz = (int)(diff.z / grid.blockSize + 0.5f);
            int startGx = startBx * VpB; int startGy = startBy * VpB; int startGz = startBz * VpB;

            for (int z = 0; z < VpB; ++z) for (int y = 0; y < VpB; ++y) for (int x = 0; x < VpB; ++x) {
                int gx = startGx + x; int gy = startGy + y; int gz = startGz + z;
                float dists[8]; glm::vec3 colors[8], normals[8];
                int insideCount = 0; bool allValid = true;

                for (int i = 0; i < 8; ++i) {
                    int nx = x + corners[i].x, ny = y + corners[i].y, nz = z + corners[i].z;
                    // Fast Path (In-Block)
                    if (nx < VpB && ny < VpB && nz < VpB) {
                        const Voxel& v = block.voxels[nz * VpB * VpB + ny * VpB + nx];
                        if (!v.valid) { allValid = false; break; }
                        dists[i] = v.signedDistance; colors[i] = v.color; normals[i] = v.normal;
                    }
                    else { // Slow Path (Neighbor)
                        int nbx = startBx, nby = startBy, nbz = startBz;
                        int nlx = nx, nly = ny, nlz = nz;
                        if (nx >= VpB) { nbx++; nlx -= VpB; } if (ny >= VpB) { nby++; nly -= VpB; } if (nz >= VpB) { nbz++; nlz -= VpB; }
                        DataBlock* nb = grid.lookup[grid.GetIndex(nbx, nby, nbz)];
                        if (!nb) { allValid = false; break; }
                        const Voxel& v = nb->voxels[nlz * VpB * VpB + nly * VpB + nlx];
                        if (!v.valid) { allValid = false; break; }
                        dists[i] = v.signedDistance; colors[i] = v.color; normals[i] = v.normal;
                    }
                    if (dists[i] < isoLevel) insideCount++;
                }

                if (allValid && insideCount > 0 && insideCount < 8) {
                    glm::vec3 avgPos(0), avgCol(0), avgNorm(0); int ints = 0;
                    for (int e = 0; e < 12; ++e) {
                        int i1 = edgePairs[e][0], i2 = edgePairs[e][1];
                        if ((dists[i1] < isoLevel) != (dists[i2] < isoLevel)) {
                            float t = (isoLevel - dists[i1]) / (dists[i2] - dists[i1]);
                            glm::vec3 p1 = grid.origin + glm::vec3((gx + corners[i1].x) * voxelSize, (gy + corners[i1].y) * voxelSize, (gz + corners[i1].z) * voxelSize);
                            glm::vec3 p2 = grid.origin + glm::vec3((gx + corners[i2].x) * voxelSize, (gy + corners[i2].y) * voxelSize, (gz + corners[i2].z) * voxelSize);
                            avgPos += glm::mix(p1, p2, t); avgCol += glm::mix(colors[i1], colors[i2], t); avgNorm += glm::mix(normals[i1], normals[i2], t);
                            ints++;
                        }
                    }
                    if (ints > 0) localVerts.push_back({ {gx,gy,gz}, {avgPos / (float)ints, glm::normalize(avgNorm), avgCol / (float)ints} });
                }
            }
            if (!localVerts.empty()) {
                std::lock_guard<std::mutex> lock(vertMutex);
                for (auto& kv : localVerts) snVertices[kv.first] = kv.second;
            }
            });

        // 2. Pass 2: Quads (Parallel)
        std::for_each(std::execution::par, grid.pool.begin(), grid.pool.end(), [&](DataBlock& block) {
            std::vector<Triangle> localTris; localTris.reserve(128);
            glm::vec3 diff = block.blockMin - grid.origin;
            int startBx = (int)(diff.x / grid.blockSize + 0.5f);
            int startBy = (int)(diff.y / grid.blockSize + 0.5f);
            int startBz = (int)(diff.z / grid.blockSize + 0.5f);
            int startGx = startBx * VpB; int startGy = startBy * VpB; int startGz = startBz * VpB;

            for (int z = 0; z < VpB; ++z) for (int y = 0; y < VpB; ++y) for (int x = 0; x < VpB; ++x) {
                const Voxel& v = block.voxels[z * VpB * VpB + y * VpB + x];
                if (!v.valid) continue;
                bool bCurr = v.signedDistance < isoLevel;
                int gx = startGx + x, gy = startGy + y, gz = startGz + z;

                auto Add = [&](GridKey k1, GridKey k2, GridKey k3, GridKey k4, bool flip) {
                    if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4)) {
                        const auto& u0 = flip ? snVertices[k4] : snVertices[k1];
                        const auto& u1 = flip ? snVertices[k3] : snVertices[k2];
                        const auto& u2 = flip ? snVertices[k2] : snVertices[k3];
                        const auto& u3 = flip ? snVertices[k1] : snVertices[k4];
                        localTris.push_back({ {u0.pos,u1.pos,u2.pos},{u0.normal,u1.normal,u2.normal},{u0.color,u1.color,u2.color} });
                        localTris.push_back({ {u0.pos,u2.pos,u3.pos},{u0.normal,u2.normal,u3.normal},{u0.color,u2.color,u3.color} });
                    }
                    };

                // X Edge
                int nx = x + 1; bool bNext = false, valid = false;
                if (nx < VpB) { if (block.voxels[z * VpB * VpB + y * VpB + nx].valid) { bNext = block.voxels[z * VpB * VpB + y * VpB + nx].signedDistance < isoLevel; valid = true; } }
                else { DataBlock* nb = grid.lookup[grid.GetIndex(startBx + 1, startBy, startBz)]; if (nb && nb->voxels[z * VpB * VpB + y * VpB].valid) { bNext = nb->voxels[z * VpB * VpB + y * VpB].signedDistance < isoLevel; valid = true; } }
                if (valid && bCurr != bNext) Add({ gx,gy - 1,gz - 1 }, { gx,gy,gz - 1 }, { gx,gy,gz }, { gx,gy - 1,gz }, !bCurr);

                // Y Edge
                int ny = y + 1; valid = false;
                if (ny < VpB) { if (block.voxels[z * VpB * VpB + ny * VpB + x].valid) { bNext = block.voxels[z * VpB * VpB + ny * VpB + x].signedDistance < isoLevel; valid = true; } }
                else { DataBlock* nb = grid.lookup[grid.GetIndex(startBx, startBy + 1, startBz)]; if (nb && nb->voxels[z * VpB * VpB + x].valid) { bNext = nb->voxels[z * VpB * VpB + x].signedDistance < isoLevel; valid = true; } }
                if (valid && bCurr != bNext) Add({ gx - 1,gy,gz - 1 }, { gx,gy,gz - 1 }, { gx,gy,gz }, { gx - 1,gy,gz }, bCurr);

                // Z Edge
                int nz = z + 1; valid = false;
                if (nz < VpB) { if (block.voxels[nz * VpB * VpB + y * VpB + x].valid) { bNext = block.voxels[nz * VpB * VpB + y * VpB + x].signedDistance < isoLevel; valid = true; } }
                else { DataBlock* nb = grid.lookup[grid.GetIndex(startBx, startBy, startBz + 1)]; if (nb && nb->voxels[y * VpB + x].valid) { bNext = nb->voxels[y * VpB + x].signedDistance < isoLevel; valid = true; } }
                if (valid && bCurr != bNext) Add({ gx - 1,gy - 1,gz }, { gx,gy - 1,gz }, { gx,gy,gz }, { gx - 1,gy,gz }, !bCurr);
            }
            if (!localTris.empty()) {
                std::lock_guard<std::mutex> lock(triMutex);
                triangles.insert(triangles.end(), localTris.begin(), localTris.end());
            }
            });
    }

    void DetectHoles()
    {
        holeEdges.clear();
        if (triangles.empty()) return;

        // 1. Vertex Welding (Vector-based mapping for speed?) 
        // Map is fine here since it runs once at the end. For max speed, using vector+sort again.

        std::unordered_map<GridKey, uint32_t, GridKeyHash> vMap;
        vMap.reserve(triangles.size());
        std::vector<glm::vec3> uniqueVerts; uniqueVerts.reserve(triangles.size());
        std::vector<uint32_t> indices; indices.reserve(triangles.size() * 3);
        float tol = 0.0001f;

        for (const auto& t : triangles) {
            for (int i = 0; i < 3; ++i) {
                GridKey key = { (int)(t.v[i].x / tol), (int)(t.v[i].y / tol), (int)(t.v[i].z / tol) };
                auto it = vMap.find(key);
                if (it == vMap.end()) {
                    uint32_t idx = (uint32_t)uniqueVerts.size();
                    vMap[key] = idx;
                    uniqueVerts.push_back(t.v[i]);
                    indices.push_back(idx);
                }
                else {
                    indices.push_back(it->second);
                }
            }
        }

        // 2. Edge Sorting (Pack into uint64 for fast sort)
        typedef uint64_t EdgeKey;
        std::vector<EdgeKey> edges; edges.reserve(indices.size());
        for (size_t i = 0; i < indices.size(); i += 3) {
            uint32_t idx[3] = { indices[i], indices[i + 1], indices[i + 2] };
            for (int k = 0; k < 3; ++k) {
                uint32_t a = idx[k], b = idx[(k + 1) % 3];
                if (a > b) std::swap(a, b);
                edges.push_back(((uint64_t)a << 32) | b);
            }
        }
        std::sort(std::execution::par, edges.begin(), edges.end());

        // 3. Find Boundary Edges (Count == 1)
        for (size_t i = 0; i < edges.size(); ) {
            size_t j = i + 1;
            while (j < edges.size() && edges[i] == edges[j]) j++;
            if (j - i == 1) { // Appears once = Not shared = Hole
                uint32_t a = (uint32_t)(edges[i] >> 32);
                uint32_t b = (uint32_t)(edges[i] & 0xFFFFFFFF);
                holeEdges.push_back({ uniqueVerts[a], uniqueVerts[b] });
            }
            i = j;
        }
    }

    void Visualize(const DirectAccessGrid* grid, bool showMesh, bool showHoles, bool showBlocks)
    {
        if (showMesh) {
            std::vector<glm::vec3> v, n, c; std::vector<uint32_t> ind;
            size_t cnt = triangles.size() * 3;
            v.reserve(cnt); n.reserve(cnt); c.reserve(cnt); ind.reserve(cnt);
            uint32_t i = 0;
            for (const auto& t : triangles) {
                v.push_back(t.v[0]); v.push_back(t.v[1]); v.push_back(t.v[2]);
                n.push_back(t.n[0]); n.push_back(t.n[1]); n.push_back(t.n[2]);
                c.push_back(t.c[0]); c.push_back(t.c[1]); c.push_back(t.c[2]);
                ind.push_back(i++); ind.push_back(i++); ind.push_back(i++);
            }
            auto ent = Feather.CreateEntity("TSDFMesh");
            auto rnd = Feather.CreateComponent<Renderable>(ent);
            rnd->Initialize(Renderable::GeometryMode::Triangles);
            rnd->AddShader(Feather.CreateShader("D", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
            rnd->AddShader(Feather.CreateShader("TwoSide", File("../../res/Shaders/TwoSide.vs"), File("../../res/Shaders/TwoSide.fs")));
            rnd->SetActiveShaderIndex(0);
            rnd->AddVertices(v); rnd->AddNormals(n); rnd->AddColors(c); rnd->AddIndices(ind);

            Feather.CreateEventCallback<KeyEvent>(ent, [](Entity e, const KeyEvent& ev) {
                if (ev.action == 0 && ev.keyCode == GLFW_KEY_GRAVE_ACCENT) Feather.GetComponent<Renderable>(e)->NextDrawingMode();
                });
        }

        if (showHoles) {
            for (const auto& edge : holeEdges) {
                VD::AddLine("Holes", edge.first, edge.second, Color::red());
            }
        }

        if (showBlocks && grid) {
            for (const auto& b : grid->pool) {
                glm::vec3 min = b.blockMin;
                glm::vec3 max = min + glm::vec3(grid->blockSize);

                glm::vec3 c[8] = {
                   {min.x, min.y, min.z}, {max.x, min.y, min.z}, {max.x, max.y, min.z}, {min.x, max.y, min.z},
                   {min.x, min.y, max.z}, {max.x, min.y, max.z}, {max.x, max.y, max.z}, {min.x, max.y, max.z}
                };

                // Bottom
                VD::AddLine("Blocks", c[0], c[1], Color::green());
                VD::AddLine("Blocks", c[1], c[2], Color::green());
                VD::AddLine("Blocks", c[2], c[3], Color::green());
                VD::AddLine("Blocks", c[3], c[0], Color::green());
                // Top
                VD::AddLine("Blocks", c[4], c[5], Color::green());
                VD::AddLine("Blocks", c[5], c[6], Color::green());
                VD::AddLine("Blocks", c[6], c[7], Color::green());
                VD::AddLine("Blocks", c[7], c[4], Color::green());
                // Sides
                VD::AddLine("Blocks", c[0], c[4], Color::green());
                VD::AddLine("Blocks", c[1], c[5], Color::green());
                VD::AddLine("Blocks", c[2], c[6], Color::green());
                VD::AddLine("Blocks", c[3], c[7], Color::green());
            }
        }
    }

    void ExportPLY(const std::string& filename) {
        PLYFormat ply;
        for (const auto& t : triangles) {
            ply.AddPoint(t.v[0].x, t.v[0].y, t.v[0].z); ply.AddNormal(t.n[0].x, t.n[0].y, t.n[0].z); ply.AddColor(t.c[0].x, t.c[0].y, t.c[0].z);
            ply.AddPoint(t.v[1].x, t.v[1].y, t.v[1].z); ply.AddNormal(t.n[1].x, t.n[1].y, t.n[1].z); ply.AddColor(t.c[1].x, t.c[1].y, t.c[1].z);
            ply.AddPoint(t.v[2].x, t.v[2].y, t.v[2].z); ply.AddNormal(t.n[2].x, t.n[2].y, t.n[2].z); ply.AddColor(t.c[2].x, t.c[2].y, t.c[2].z);
            size_t s = ply.GetPoints().size() / 3; ply.AddFace(s - 3, s - 2, s - 1);
        }
        ply.Serialize(filename);
    }
};

// Processor struct definition
struct Processor {
    DirectAccessGrid grid;
    MeshGenerator meshGen;

    // [Optimization] Spinlock wrapper (replaces DataBlock mutex)
    // It is recommended to modify DataBlock definition for maximum speed.
    // However, here we stick to existing Mutex for simplicity, focusing on logic optimization (8-block limit).

    void Process(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const std::vector<glm::vec3>& colors, const glm::vec3& aabbMin, const glm::vec3& aabbMax) {
        grid.Initialize(aabbMin - glm::vec3(1), aabbMax + glm::vec3(1));
        float truncDist = std::max(Configuration.voxelSize * 4.0f, 0.15f);
        float voxelSize = Configuration.voxelSize;

        // ----------------------------------------------------------------
        // Pass 1: Mark Active Blocks (Precise Range Check)
        // ----------------------------------------------------------------
        TS(Pass1_Alloc);
        std::vector<uint8_t> flags(grid.totalCells, 0);
        size_t nPts = points.size();
        std::vector<size_t> idx(nPts); std::iota(idx.begin(), idx.end(), 0);

        std::for_each(std::execution::par, idx.begin(), idx.end(), [&](size_t i) {
            glm::vec3 p = points[i];

            // Point influence range (AABB) calculation
            glm::vec3 minP = p - glm::vec3(truncDist);
            glm::vec3 maxP = p + glm::vec3(truncDist);

            // Block indices covered by range
            glm::vec3 minDiff = minP - grid.origin;
            glm::vec3 maxDiff = maxP - grid.origin;

            int minBx = (int)std::floor(minDiff.x / grid.blockSize);
            int maxBx = (int)std::floor(maxDiff.x / grid.blockSize);
            int minBy = (int)std::floor(minDiff.y / grid.blockSize);
            int maxBy = (int)std::floor(maxDiff.y / grid.blockSize);
            int minBz = (int)std::floor(minDiff.z / grid.blockSize);
            int maxBz = (int)std::floor(maxDiff.z / grid.blockSize);

            // Mark only blocks within range (Max 2x2x2 = 8)
            for (int bz = minBz; bz <= maxBz; ++bz) {
                for (int by = minBy; by <= maxBy; ++by) {
                    for (int bx = minBx; bx <= maxBx; ++bx) {
                        int index = grid.GetIndex(bx, by, bz);
                        if (index != -1) flags[index] = 1;
                    }
                }
            }
            });

        // Allocation
        int count = 0; for (uint8_t f : flags) if (f) count++;
        grid.pool.resize(count);
        int poolIdx = 0;
        for (int i = 0; i < grid.totalCells; ++i) {
            if (flags[i]) {
                DataBlock& b = grid.pool[poolIdx]; b.Initialize();
                int bz = i / (grid.dim.x * grid.dim.y); int rem = i % (grid.dim.x * grid.dim.y);
                int by = rem / grid.dim.x; int bx = rem % grid.dim.x;
                b.blockMin = grid.origin + glm::vec3((float)bx, (float)by, (float)bz) * grid.blockSize;
                grid.lookup[i] = &b; poolIdx++;
            }
        }
        TE(Pass1_Alloc);

        // ----------------------------------------------------------------
        // Pass 2: Occupy (Precise Range & Loop Unrolling)
        // ----------------------------------------------------------------
        TS(Occupy);

        // Pre-calculated constant
        float truncDistSq = truncDist * truncDist;

        std::for_each(std::execution::par, idx.begin(), idx.end(), [&](size_t i) {
            glm::vec3 p = points[i];
            glm::vec3 n = normals.empty() ? glm::vec3(0, 1, 0) : normals[i];
            glm::vec3 c = colors.empty() ? glm::vec3(1) : colors[i]; if (c.r > 1) c /= 255.0f;

            // Accurate block range calculation same as Pass 1
            glm::vec3 minP = p - glm::vec3(truncDist);
            glm::vec3 maxP = p + glm::vec3(truncDist);
            glm::vec3 minDiff = minP - grid.origin;
            glm::vec3 maxDiff = maxP - grid.origin;

            int minBx = (int)std::floor(minDiff.x / grid.blockSize);
            int maxBx = (int)std::floor(maxDiff.x / grid.blockSize);
            int minBy = (int)std::floor(minDiff.y / grid.blockSize);
            int maxBy = (int)std::floor(maxDiff.y / grid.blockSize);
            int minBz = (int)std::floor(minDiff.z / grid.blockSize);
            int maxBz = (int)std::floor(maxDiff.z / grid.blockSize);

            // Iterate only relevant blocks
            for (int bz = minBz; bz <= maxBz; ++bz) {
                for (int by = minBy; by <= maxBy; ++by) {
                    for (int bx = minBx; bx <= maxBx; ++bx) {
                        DataBlock* b = grid.lookup[grid.GetIndex(bx, by, bz)];
                        if (!b) continue;

                        // Local Voxel Range calculation
                        glm::vec3 lPos = p - b->blockMin;

                        // Range clamping
                        int minX = std::max(0, (int)((lPos.x - truncDist) / voxelSize));
                        int maxX = std::min(VpB, (int)((lPos.x + truncDist) / voxelSize) + 1);
                        int minY = std::max(0, (int)((lPos.y - truncDist) / voxelSize));
                        int maxY = std::min(VpB, (int)((lPos.y + truncDist) / voxelSize) + 1);
                        int minZ = std::max(0, (int)((lPos.z - truncDist) / voxelSize));
                        int maxZ = std::min(VpB, (int)((lPos.z + truncDist) / voxelSize) + 1);

                        if (minX >= maxX || minY >= maxY || minZ >= maxZ) continue;

                        // Lock Block
                        std::lock_guard<std::mutex> lk(b->blockMutex);

                        for (int z = minZ; z < maxZ; ++z) {
                            for (int y = minY; y < maxY; ++y) {
                                float vz = (z + 0.5f) * voxelSize;
                                float vy = (y + 0.5f) * voxelSize;
                                float dz = vz - lPos.z;
                                float dy = vy - lPos.y;
                                float dzySq = dz * dz + dy * dy;

                                int baseIdx = z * VpB * VpB + y * VpB;

                                for (int x = minX; x < maxX; ++x) {
                                    float vx = (x + 0.5f) * voxelSize;
                                    float dx = vx - lPos.x;
                                    float distSq = dzySq + dx * dx;

                                    if (distSq > truncDistSq) continue;

                                    float d = std::sqrt(distSq);
                                    float w = 1.0f - (d / truncDist);

                                    // SDF Calculation (p - vc = - (vc - p))
                                    glm::vec3 dir = p - (b->blockMin + glm::vec3(vx, vy, vz));
                                    float sdf = glm::clamp(glm::dot(dir, -n), -truncDist, truncDist);

                                    Voxel& v = b->voxels[baseIdx + x];

                                    if (!v.valid) { // Recommended to use valid flag instead of weight check
                                        v.signedDistance = sdf;
                                        v.color = c;
                                        v.normal = n;
                                        v.weight = w;
                                        v.valid = 1;
                                    }
                                    else {
                                        float nw = v.weight + w;
                                        float invNw = 1.0f / nw;
                                        v.signedDistance = (v.signedDistance * v.weight + sdf * w) * invNw;
                                        v.color = (v.color * v.weight + c * w) * invNw;
                                        v.normal = (v.normal * v.weight + n * w) * invNw;
                                        v.weight = nw;
                                    }
                                }
                            }
                        }
                    }
                }
            }
            });
        TE(Occupy);

        TS(MeshGen);
        meshGen.Generate(grid);
        meshGen.DetectHoles();
        TE(MeshGen);
    }
};

int main(int argc, char** argv) {
    std::cout << "AppFeather - Hybrid Ultimate" << std::endl;
    Feather.Initialize(1920, 1080);
    Feather.SetConsoleWindowIndex(3);
    Feather.SetMainWindowIndex(2);
    auto w = Feather.GetFeatherWindow();

#pragma region AppMain
    {
        auto appMain = Feather.CreateEntity("AppMain");
        Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
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

        Feather.CreateEventCallback<FrameBufferResizeEvent>(cam, [pcam](Entity entity, const FrameBufferResizeEvent& event) {
            auto window = Feather.GetFeatherWindow();
            auto aspectRatio = (f32)window->GetWidth() / (f32)window->GetHeight();
            pcam->SetAspectRatio(aspectRatio);
            });

        Feather.CreateEventCallback<KeyEvent>(cam, [](Entity entity, const KeyEvent& event) {
            Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnKey(event);
            });

        Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity entity, const MousePositionEvent& event) {
            Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMousePosition(event);
            });

        Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity entity, const MouseButtonEvent& event) {
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

        Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) {
            Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event);
            });
    }
#pragma endregion

    Feather.AddOnInitializeCallback([&]() {
        TS(PLYLoading);
        PLYFormat ply;
        if (!ply.Deserialize("D:\\Debug\\PLY\\inputA.ply")) return;
        std::vector<glm::vec3> p, n, c;
        size_t cnt = ply.GetPoints().size() / 3;
        p.reserve(cnt); if (!ply.GetNormals().empty()) n.reserve(cnt); if (!ply.GetColors().empty()) c.reserve(cnt);
        auto& rp = ply.GetPoints(); auto& rn = ply.GetNormals(); auto& rc = ply.GetColors(); bool ha = ply.UseAlpha();
        for (size_t i = 0; i < cnt; ++i) {
            float x = rp[i * 3], y = rp[i * 3 + 1], z = rp[i * 3 + 2];
            if (x >= Configuration.filterMin.x && x <= Configuration.filterMax.x && y >= Configuration.filterMin.y && y <= Configuration.filterMax.y && z >= Configuration.filterMin.z && z <= Configuration.filterMax.z) {
                p.push_back({ x,y,z });
                if (!rn.empty()) n.push_back({ rn[i * 3],rn[i * 3 + 1],rn[i * 3 + 2] });
                if (!rc.empty()) c.push_back(ha ? glm::vec3(rc[i * 4], rc[i * 4 + 1], rc[i * 4 + 2]) : glm::vec3(rc[i * 3], rc[i * 3 + 1], rc[i * 3 + 2]));
            }
        }
        TE(PLYLoading);

        TS(Processing);
        auto [mn, mm, mz] = ply.GetAABBMin();
        auto [xn, xm, xz] = ply.GetAABBMax();

        static Processor proc;
        proc.Process(p, n, c, glm::vec3(mn, mm, mz), glm::vec3(xn, xm, xz));
        TE(Processing);

        // Pass grid pointer to visualize blocks
        proc.meshGen.Visualize(&proc.grid, true, true, true);
        //proc.meshGen.ExportPLY("D:\\Debug\\PLY\\output.ply");

        alog("Total Blocks: %s\n", FormatWithCommas(proc.grid.pool.size()).c_str());
        alog("Total Memory: %s bytes\n", FormatWithCommas(proc.grid.pool.size() * sizeof(DataBlock)).c_str());

#pragma region Status Panel
        {
            auto gui = Feather.GetRegistry().create();
            auto statusPanel = Feather.GetRegistry().emplace<StatusPanel>(gui);

            Feather.CreateEventCallback<MousePositionEvent>(gui, [](Entity entity, const MousePositionEvent& event) {
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