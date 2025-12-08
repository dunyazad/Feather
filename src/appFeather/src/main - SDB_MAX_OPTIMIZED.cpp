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
#include <atomic>

struct Configuration
{
    const float voxelSize = 0.2f;
    const int sdfOffset = 1;
    glm::vec3 filterMin = glm::vec3(-10.0f, -10.0f, -10.0f);
    glm::vec3 filterMax = glm::vec3(10.0f, 10.0f, 10.0f);
} Configuration;

using VD = VisualDebugging;
using namespace libRxTx;

const int VpB = 8; // 8x8x8 voxels per block

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
    std::mutex blockMutex;

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
    std::unordered_map<DataBlockKey, std::unique_ptr<DataBlock>> dataBlocks;

    // Helper: Block Size Cache
    float blockSize = voxelSize * VpB;

    Voxel* GetVoxelByIndex(int gx, int gy, int gz)
    {
        int bx = (int)floor((float)gx / VpB);
        int by = (int)floor((float)gy / VpB);
        int bz = (int)floor((float)gz / VpB);

        glm::vec3 blockMin = gridOrigin + glm::vec3((float)bx * blockSize, (float)by * blockSize, (float)bz * blockSize);
        auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSize);

        auto it = dataBlocks.find(key);
        if (it == dataBlocks.end()) return nullptr;

        int lx = gx % VpB; if (lx < 0) lx += VpB;
        int ly = gy % VpB; if (ly < 0) ly += VpB;
        int lz = gz % VpB; if (lz < 0) lz += VpB;

        return &it->second->voxels[lz * VpB * VpB + ly * VpB + lx];
    }

    // [최적화] IO 분리: 이미 로드된 데이터를 받아 처리 (재로딩 방지)
    void FromPointsData(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const std::vector<glm::vec3>& colors, const glm::vec3& aabbMin)
    {
        // 1. Setup Grid Origin
        gridOrigin.x = std::floor(aabbMin.x / blockSize) * blockSize;
        gridOrigin.y = std::floor(aabbMin.y / blockSize) * blockSize;
        gridOrigin.z = std::floor(aabbMin.z / blockSize) * blockSize;

        TS(Occupy);

        float truncDist = std::max(voxelSize * 4.0f, 0.15f);

        // Prepare Parallel Indices
        size_t numPoints = points.size();
        std::vector<size_t> indices(numPoints);
        std::iota(indices.begin(), indices.end(), 0);

        // ------------------------------------------------------------------
        // Pass 1: Smart Allocation (Parallel + Boundary Check)
        // 포인트가 블록 경계에 있을 때만 이웃 블록을 할당하여 메모리 절약
        // ------------------------------------------------------------------
        {
            TS(Pass1_Alloc);

            using KeyPair = std::pair<DataBlockKey, glm::vec3>;

            std::vector<KeyPair> keysToAllocate;
            keysToAllocate.reserve(numPoints * 2);
            std::mutex vecMutex;

            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
                {
                    std::vector<KeyPair> localKeys;
                    localKeys.reserve(8);

                    glm::vec3 p = points[i];
                    glm::vec3 vecFromOrigin = p - gridOrigin;

                    int centerGx = (int)std::floor(vecFromOrigin.x / voxelSize);
                    int centerGy = (int)std::floor(vecFromOrigin.y / voxelSize);
                    int centerGz = (int)std::floor(vecFromOrigin.z / voxelSize);

                    // 현재 포인트가 속한 중심 블록
                    int bx = centerGx / VpB;
                    int by = centerGy / VpB;
                    int bz = centerGz / VpB;

                    glm::vec3 blockMin = gridOrigin + glm::vec3((float)bx * blockSize, (float)by * blockSize, (float)bz * blockSize);
                    auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSize);
                    localKeys.push_back({ key, blockMin });

                    // [최적화] Boundary Check
                    // 포인트가 블록 경계(truncDist) 근처에 있을 때만 해당 방향의 이웃 블록 추가
                    glm::vec3 localP = p - blockMin;

                    // 각 축별로 경계 침범 여부 확인 (-방향, +방향)
                    // truncDist 여유분을 두어 SDF가 퍼질 수 있는 공간 확보
                    float margin = truncDist + voxelSize * 0.5f;

                    bool nearX_Neg = localP.x < margin;
                    bool nearX_Pos = localP.x > blockSize - margin;
                    bool nearY_Neg = localP.y < margin;
                    bool nearY_Pos = localP.y > blockSize - margin;
                    bool nearZ_Neg = localP.z < margin;
                    bool nearZ_Pos = localP.z > blockSize - margin;

                    if (nearX_Neg || nearX_Pos || nearY_Neg || nearY_Pos || nearZ_Neg || nearZ_Pos)
                    {
                        // 필요한 이웃만 선별적으로 추가
                        int dx_min = nearX_Neg ? -1 : 0;
                        int dx_max = nearX_Pos ? 1 : 0;
                        int dy_min = nearY_Neg ? -1 : 0;
                        int dy_max = nearY_Pos ? 1 : 0;
                        int dz_min = nearZ_Neg ? -1 : 0;
                        int dz_max = nearZ_Pos ? 1 : 0;

                        for (int dz = dz_min; dz <= dz_max; ++dz) {
                            for (int dy = dy_min; dy <= dy_max; ++dy) {
                                for (int dx = dx_min; dx <= dx_max; ++dx) {
                                    if (dx == 0 && dy == 0 && dz == 0) continue;

                                    glm::vec3 nbMin = gridOrigin + glm::vec3((float)(bx + dx) * blockSize, (float)(by + dy) * blockSize, (float)(bz + dz) * blockSize);
                                    auto nKey = Morton3D::EncodeFromVec3(nbMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSize);
                                    localKeys.push_back({ nKey, nbMin });
                                }
                            }
                        }
                    }

                    if (!localKeys.empty())
                    {
                        std::lock_guard<std::mutex> lock(vecMutex);
                        keysToAllocate.insert(keysToAllocate.end(), localKeys.begin(), localKeys.end());
                    }
                });

            // 정렬 및 중복 제거 (매우 빠름)
            std::sort(std::execution::par, keysToAllocate.begin(), keysToAllocate.end(),
                [](const KeyPair& a, const KeyPair& b) { return a.first < b.first; });

            auto last = std::unique(std::execution::par, keysToAllocate.begin(), keysToAllocate.end(),
                [](const KeyPair& a, const KeyPair& b) { return a.first == b.first; });

            keysToAllocate.erase(last, keysToAllocate.end());

            // 실제 메모리 할당
            for (const auto& kv : keysToAllocate)
            {
                if (dataBlocks.find(kv.first) == dataBlocks.end())
                {
                    dataBlocks[kv.first] = std::make_unique<DataBlock>();
                    dataBlocks[kv.first]->Initialize();
                    dataBlocks[kv.first]->blockMin = kv.second;
                }
            }
            TE(Pass1_Alloc);
        }

        // ------------------------------------------------------------------
        // Pass 2: Update Voxels (Parallel)
        // ------------------------------------------------------------------
        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
            {
                auto p = points[i];
                glm::vec3 n = (normals.empty()) ? glm::vec3(0, 1, 0) : normals[i];
                glm::vec3 c = (colors.empty()) ? glm::vec3(1, 1, 1) : colors[i];
                if (c.r > 1.0f) c /= 255.0f;

                glm::vec3 vecFromOrigin = p - gridOrigin;
                int centerGx = (int)std::floor(vecFromOrigin.x / voxelSize);
                int centerGy = (int)std::floor(vecFromOrigin.y / voxelSize);
                int centerGz = (int)std::floor(vecFromOrigin.z / voxelSize);

                // Thread-Local Cache
                DataBlockKey lastKey = (DataBlockKey)-1;
                DataBlock* cachedBlock = nullptr;

                // 3x3x3 이웃 블록 순회
                for (int dz = -1; dz <= 1; ++dz) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dx = -1; dx <= 1; ++dx) {
                            int gx = centerGx + dx;
                            int gy = centerGy + dy;
                            int gz = centerGz + dz;

                            // 거리 체크로 불필요한 연산 방지
                            glm::vec3 voxelCenter = gridOrigin + glm::vec3((gx + 0.5f) * voxelSize, (gy + 0.5f) * voxelSize, (gz + 0.5f) * voxelSize);
                            float dist = glm::distance(p, voxelCenter);
                            if (dist > truncDist) continue;

                            int bx = (int)floor((float)gx / VpB);
                            int by = (int)floor((float)gy / VpB);
                            int bz = (int)floor((float)gz / VpB);

                            float currBlockSize = blockSize;
                            glm::vec3 blockMin = gridOrigin + glm::vec3(bx * currBlockSize, by * currBlockSize, bz * currBlockSize);
                            auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, currBlockSize);

                            DataBlock* targetBlock = nullptr;
                            if (key == lastKey && cachedBlock) targetBlock = cachedBlock;
                            else {
                                auto it = dataBlocks.find(key);
                                if (it != dataBlocks.end()) {
                                    targetBlock = it->second.get();
                                    lastKey = key; cachedBlock = targetBlock;
                                }
                            }

                            if (targetBlock) {
                                int lx = gx % VpB; if (lx < 0) lx += VpB;
                                int ly = gy % VpB; if (ly < 0) ly += VpB;
                                int lz = gz % VpB; if (lz < 0) lz += VpB;

                                float weight = 1.0f - (dist / truncDist);
                                float sdf = glm::clamp(glm::dot(voxelCenter - p, n), -truncDist, truncDist);

                                // Block 단위 Lock 사용
                                std::lock_guard<std::mutex> lock(targetBlock->blockMutex);
                                Voxel& voxel = targetBlock->voxels[lz * VpB * VpB + ly * VpB + lx];

                                if (voxel.weight <= 0.0001f) {
                                    voxel.signedDistance = sdf; voxel.color = c; voxel.normal = n; voxel.weight = weight; voxel.valid = true;
                                }
                                else {
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

struct Triangle { glm::vec3 v[3]; glm::vec3 n[3]; glm::vec3 c[3]; };

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

        // Block Iteration Vector (병렬 처리용)
        std::vector<DataBlock*> blocks;
        blocks.reserve(sdb.dataBlocks.size());
        for (auto& pair : sdb.dataBlocks) blocks.push_back(pair.second.get());

        // Memory Reservation
        size_t estTris = blocks.size() * 96;
        triangles.reserve(estTris);

        // Concurrent Map for Vertices
        std::unordered_map<GridKey, SNVertex, GridKeyHash> snVertices;
        snVertices.reserve(estTris * 0.6f);
        snVertices.max_load_factor(0.7f);
        std::mutex vertexMutex;
        std::mutex triMutex;

        const glm::ivec3 corners[8] = { {0,0,0}, {1,0,0}, {1,0,1}, {0,0,1}, {0,1,0}, {1,1,0}, {1,1,1}, {0,1,1} };
        const int edgePairs[12][2] = { {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7} };

        // -------------------------------------------------------
        // Pass 1: Generate Vertices (Parallel)
        // -------------------------------------------------------
        std::for_each(std::execution::par, blocks.begin(), blocks.end(), [&](DataBlock* block)
            {
                std::vector<std::pair<GridKey, SNVertex>> localVerts;
                localVerts.reserve(64);

                glm::vec3 diff = block->blockMin - sdb.gridOrigin;
                int startGx = (int)(diff.x / sdb.voxelSize + 0.5f);
                int startGy = (int)(diff.y / sdb.voxelSize + 0.5f);
                int startGz = (int)(diff.z / sdb.voxelSize + 0.5f);

                for (int z = 0; z < VpB; ++z) {
                    for (int y = 0; y < VpB; ++y) {
                        for (int x = 0; x < VpB; ++x) {
                            int gx = startGx + x; int gy = startGy + y; int gz = startGz + z;

                            float dists[8]; glm::vec3 colors[8], normals[8];
                            int insideCount = 0; bool allValid = true;

                            for (int i = 0; i < 8; ++i) {
                                const auto* v = sdb.GetVoxelByIndex(gx + corners[i].x, gy + corners[i].y, gz + corners[i].z);
                                if (!v || !v->valid) { allValid = false; break; }
                                dists[i] = v->signedDistance; colors[i] = v->color; normals[i] = v->normal;
                                if (dists[i] < isoLevel) insideCount++;
                            }

                            if (!allValid || insideCount == 0 || insideCount == 8) continue;

                            glm::vec3 avgPos(0.0f), avgColor(0.0f), avgNormal(0.0f);
                            int intersections = 0;
                            for (int e = 0; e < 12; ++e) {
                                int idx1 = edgePairs[e][0]; int idx2 = edgePairs[e][1];
                                if ((dists[idx1] < isoLevel) != (dists[idx2] < isoLevel)) {
                                    float t = (isoLevel - dists[idx1]) / (dists[idx2] - dists[idx1]);
                                    glm::vec3 p1 = sdb.gridOrigin + glm::vec3(gx + corners[idx1].x, gy + corners[idx1].y, gz + corners[idx1].z) * sdb.voxelSize;
                                    glm::vec3 p2 = sdb.gridOrigin + glm::vec3(gx + corners[idx2].x, gy + corners[idx2].y, gz + corners[idx2].z) * sdb.voxelSize;
                                    avgPos += glm::mix(p1, p2, t);
                                    avgColor += glm::mix(colors[idx1], colors[idx2], t);
                                    avgNormal += glm::mix(normals[idx1], normals[idx2], t);
                                    intersections++;
                                }
                            }

                            if (intersections > 0) {
                                SNVertex v;
                                v.pos = avgPos / (float)intersections;
                                v.color = avgColor / (float)intersections;
                                v.normal = glm::normalize(avgNormal);
                                localVerts.push_back({ {gx, gy, gz}, v });
                            }
                        }
                    }
                }
                if (!localVerts.empty()) {
                    std::lock_guard<std::mutex> lock(vertexMutex);
                    for (const auto& kv : localVerts) snVertices[kv.first] = kv.second;
                }
            });

        // -------------------------------------------------------
        // Pass 2: Generate Quads (Parallel)
        // -------------------------------------------------------
        std::for_each(std::execution::par, blocks.begin(), blocks.end(), [&](DataBlock* block)
            {
                std::vector<Triangle> localTris;
                localTris.reserve(128);

                glm::vec3 diff = block->blockMin - sdb.gridOrigin;
                int startGx = (int)(diff.x / sdb.voxelSize + 0.5f);
                int startGy = (int)(diff.y / sdb.voxelSize + 0.5f);
                int startGz = (int)(diff.z / sdb.voxelSize + 0.5f);

                for (int z = 0; z < VpB; ++z) {
                    for (int y = 0; y < VpB; ++y) {
                        for (int x = 0; x < VpB; ++x) {
                            int gx = startGx + x; int gy = startGy + y; int gz = startGz + z;
                            const auto* vCurr = sdb.GetVoxelByIndex(gx, gy, gz);
                            if (!vCurr || !vCurr->valid) continue;
                            bool bCurr = vCurr->signedDistance < isoLevel;

                            auto AddQuadLoc = [&](const GridKey& k1, const GridKey& k2, const GridKey& k3, const GridKey& k4, bool flip) {
                                if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4)) {
                                    const auto& v0 = flip ? snVertices[k4] : snVertices[k1];
                                    const auto& v1 = flip ? snVertices[k3] : snVertices[k2];
                                    const auto& v2 = flip ? snVertices[k2] : snVertices[k3];
                                    const auto& v3 = flip ? snVertices[k1] : snVertices[k4];
                                    Triangle t1, t2;
                                    t1.v[0] = v0.pos; t1.v[1] = v1.pos; t1.v[2] = v2.pos;
                                    t1.c[0] = v0.color; t1.c[1] = v1.color; t1.c[2] = v2.color;
                                    t1.n[0] = v0.normal; t1.n[1] = v1.normal; t1.n[2] = v2.normal;
                                    t2.v[0] = v0.pos; t2.v[1] = v2.pos; t2.v[2] = v3.pos;
                                    t2.c[0] = v0.color; t2.c[1] = v2.color; t2.c[2] = v3.color;
                                    t2.n[0] = v0.normal; t2.n[1] = v2.normal; t2.n[2] = v3.normal;
                                    localTris.push_back(t1); localTris.push_back(t2);
                                }
                                };

                            const auto* vX = sdb.GetVoxelByIndex(gx + 1, gy, gz);
                            if (vX && vX->valid && (bCurr != (vX->signedDistance < isoLevel)))
                                AddQuadLoc({ gx, gy - 1, gz - 1 }, { gx, gy, gz - 1 }, { gx, gy, gz }, { gx, gy - 1, gz }, !bCurr);

                            const auto* vY = sdb.GetVoxelByIndex(gx, gy + 1, gz);
                            if (vY && vY->valid && (bCurr != (vY->signedDistance < isoLevel)))
                                AddQuadLoc({ gx - 1, gy, gz - 1 }, { gx, gy, gz - 1 }, { gx, gy, gz }, { gx - 1, gy, gz }, bCurr);

                            const auto* vZ = sdb.GetVoxelByIndex(gx, gy, gz + 1);
                            if (vZ && vZ->valid && (bCurr != (vZ->signedDistance < isoLevel)))
                                AddQuadLoc({ gx - 1, gy - 1, gz }, { gx, gy - 1, gz }, { gx, gy, gz }, { gx - 1, gy, gz }, !bCurr);
                        }
                    }
                }
                if (!localTris.empty()) {
                    std::lock_guard<std::mutex> lock(triMutex);
                    triangles.insert(triangles.end(), localTris.begin(), localTris.end());
                }
            });
    }

    void Visualize(bool showMesh, bool showHoles)
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
            rnd->AddShader(Feather.CreateShader("Default", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
            rnd->AddShader(Feather.CreateShader("TwoSide", File("../../res/Shaders/TwoSide.vs"), File("../../res/Shaders/TwoSide.fs")));
            rnd->SetActiveShaderIndex(0);
            rnd->AddVertices(v); rnd->AddNormals(n); rnd->AddColors(c); rnd->AddIndices(ind);

            Feather.CreateEventCallback<KeyEvent>(ent, [](Entity e, const KeyEvent& ev) {
                if (ev.action == 0 && ev.keyCode == GLFW_KEY_GRAVE_ACCENT) Feather.GetComponent<Renderable>(e)->NextDrawingMode();
                });
        }
        if (showHoles) {
            for (const auto& edge : holeEdges) VD::AddLine("Holes", edge.first, edge.second, Color::red());
        }
    }

    void ExportPLY(const std::string& filename)
    {
        PLYFormat ply;
        // Fast Export
        for (const auto& t : triangles) {
            ply.AddPoint(t.v[0].x, t.v[0].y, t.v[0].z); ply.AddNormal(t.n[0].x, t.n[0].y, t.n[0].z); ply.AddColor(t.c[0].x, t.c[0].y, t.c[0].z);
            ply.AddPoint(t.v[1].x, t.v[1].y, t.v[1].z); ply.AddNormal(t.n[1].x, t.n[1].y, t.n[1].z); ply.AddColor(t.c[1].x, t.c[1].y, t.c[1].z);
            ply.AddPoint(t.v[2].x, t.v[2].y, t.v[2].z); ply.AddNormal(t.n[2].x, t.n[2].y, t.n[2].z); ply.AddColor(t.c[2].x, t.c[2].y, t.c[2].z);
            size_t s = ply.GetPoints().size() / 3;
            ply.AddFace((uint32_t)s - 3, (uint32_t)s - 2, (uint32_t)s - 1);
        }
        if (ply.Serialize(filename)) printf("Exported PLY: %s (Tris: %zu)\n", filename.c_str(), triangles.size());
    }

    void DetectHoles() {
        // 기존 구현과 동일
        float tol = 0.0001f;
        std::map<std::pair<int, int>, int> edges;
        std::unordered_map<GridKey, int, GridKeyHash> vMap;
        vMap.reserve(triangles.size());
        int vCount = 0;
        std::vector<int> triIndices; triIndices.reserve(triangles.size() * 3);
        std::vector<glm::vec3> tempVerts; tempVerts.reserve(triangles.size());

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
            if (kv.second == 1) holeEdges.push_back({ tempVerts[kv.first.first], tempVerts[kv.first.second] });
        }
    }
};

static inline std::string FormatWithCommas(size_t value)
{
    std::string numStr = std::to_string(value);
    int insertPosition = static_cast<int>(numStr.length()) - 3;
    while (insertPosition > 0) { numStr.insert(insertPosition, ","); insertPosition -= 3; }
    return numStr;
}

int main(int argc, char** argv)
{
    std::cout << "AppFeather - Final Optimized" << std::endl;
    Feather.Initialize(1920, 1080);
    Feather.SetConsoleWindowIndex(3); Feather.SetMainWindowIndex(2);
    auto w = Feather.GetFeatherWindow();

    // AppMain & Camera Setup
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
            pcam->SetAspectRatio((f32)Feather.GetFeatherWindow()->GetWidth() / (f32)Feather.GetFeatherWindow()->GetHeight());
            });
        Feather.CreateEventCallback<KeyEvent>(cam, [](Entity entity, const KeyEvent& event) { Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnKey(event); });
        Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity entity, const MousePositionEvent& event) { Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMousePosition(event); });
        Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity entity, const MouseButtonEvent& event) {
            Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMouseButton(event);
            if (event.button == 0 && event.action == 0) {
                int w, h; glfwGetFramebufferSize(Feather.GetFeatherWindow()->GetGLFWwindow(), &w, &h);
                float d = 0; glReadPixels((int)event.xpos, h - (int)event.ypos - 1, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &d);
                printf("Depth: %f\n", d);
            }
            });
        Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) { Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event); });
    }

    Feather.AddOnInitializeCallback([&]()
        {
            // 1. Load PLY Once (IO Optimization)
            TS(PLYLoading);
            PLYFormat ply;
            if (!ply.Deserialize("D:\\Debug\\PLY\\inputA.ply")) { printf("Failed to load PLY.\n"); return; }

            // Manual Filtering to avoid internal copy
            std::vector<glm::vec3> points, normals, colors;
            size_t rawCount = ply.GetPoints().size() / 3;
            points.reserve(rawCount);
            if (!ply.GetNormals().empty()) normals.reserve(rawCount);
            if (!ply.GetColors().empty()) colors.reserve(rawCount);

            auto& rawPts = ply.GetPoints(); auto& rawNorms = ply.GetNormals(); auto& rawCols = ply.GetColors();
            bool hasN = !rawNorms.empty(); bool hasC = !rawCols.empty(); bool useAlpha = ply.UseAlpha();

            for (size_t i = 0; i < rawCount; ++i) {
                float x = rawPts[i * 3], y = rawPts[i * 3 + 1], z = rawPts[i * 3 + 2];
                if (x >= Configuration.filterMin.x && x <= Configuration.filterMax.x &&
                    y >= Configuration.filterMin.y && y <= Configuration.filterMax.y &&
                    z >= Configuration.filterMin.z && z <= Configuration.filterMax.z) {
                    points.push_back({ x,y,z });
                    if (hasN) normals.push_back({ rawNorms[i * 3], rawNorms[i * 3 + 1], rawNorms[i * 3 + 2] });
                    if (hasC) {
                        if (useAlpha) colors.push_back({ rawCols[i * 4], rawCols[i * 4 + 1], rawCols[i * 4 + 2] });
                        else colors.push_back({ rawCols[i * 3], rawCols[i * 3 + 1], rawCols[i * 3 + 2] });
                    }
                }
            }
            TE(PLYLoading);

            // 2. Process
            auto [minx, miny, minz] = ply.GetAABBMin();
            glm::vec3 aabbMin(minx - 1.0f, miny - 1.0f, minz - 1.0f);

            SparseDataBlock sdb;
            // Pass data directly (No double loading)
            sdb.FromPointsData(points, normals, colors, aabbMin);

            TS(MeshGeneration);
            static MeshGenerator meshGen;
            meshGen.Generate(sdb);
            meshGen.DetectHoles();
            TE(MeshGeneration);

            meshGen.Visualize(true, true);
            meshGen.ExportPLY("D:\\Debug\\PLY\\output.ply");

            alog("Total DataBlocks : %s\n", FormatWithCommas(sdb.dataBlocks.size()).c_str());
            alog("DataBlock Size : %zd\n", sizeof(DataBlock));
            alog("Total Memory : %s bytes\n", FormatWithCommas(sdb.dataBlocks.size() * sizeof(DataBlock)).c_str());

            // Status Panel
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
