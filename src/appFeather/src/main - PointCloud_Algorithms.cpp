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
    const float voxelSize = 0.1f;
    const int sdfOffset = 1;
    //glm::vec3 filterMin = glm::vec3(-10.0f, -10.0f, -10.0f);
    //glm::vec3 filterMax = glm::vec3(10.0f, 10.0f, 10.0f);
    glm::vec3 filterMin = glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
    glm::vec3 filterMax = glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX);
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
    int clusterId = -1;
    float divergence = 0.0f;
};

struct DataBlock
{
    glm::vec3 blockMin = glm::zero<glm::vec3>();
    Voxel voxels[VpB * VpB * VpB];
    std::mutex blockMutex;

    void Initialize()
    {
        //memset(voxels, 0, sizeof(Voxel) * VpB * VpB * VpB);

        for (int i = 0; i < VpB * VpB * VpB; ++i)
        {
            voxels[i] = Voxel();
        }
    }
};

typedef uint64_t DataBlockKey;

struct PointCloudClusterer
{
    std::vector<int> pointClusterIds;
    std::vector<glm::vec3> clusterColors;
    std::vector<float> pointDivergences;

    struct PointGridKey
    {
        int x, y, z;
        bool operator==(const PointGridKey& o) const
        {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct PointGridKeyHash
    {
        size_t operator()(const PointGridKey& k) const
        {
            return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
        }
    };

    void Process(
        const std::vector<glm::vec3>& points,
        const std::vector<glm::vec3>& normals,
        float radius,
        float angleThresholdDeg,
        float maxDivergence = 0.05f)
    {
        TS(PointClustering);
        size_t numPoints = points.size();
        pointClusterIds.assign(numPoints, -1);
        pointDivergences.assign(numPoints, 0.0f);

        if (points.empty() || normals.empty()) return;

        float thresholdDot = cos(glm::radians(angleThresholdDeg));
        float cellSize = radius;

        // 1. Build Spatial Index (Grid)
        std::unordered_map<PointGridKey, std::vector<int>, PointGridKeyHash> grid;
        grid.reserve(numPoints);

        for (int i = 0; i < (int)numPoints; ++i)
        {
            PointGridKey key = {
                (int)std::floor(points[i].x / cellSize),
                (int)std::floor(points[i].y / cellSize),
                (int)std::floor(points[i].z / cellSize)
            };
            grid[key].push_back(i);
        }

        // 2. Calculate Divergences (Parallel)
        std::vector<int> indices(numPoints);
        std::iota(indices.begin(), indices.end(), 0);

        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
            {
                glm::vec3 currPos = points[i];
                glm::vec3 currNorm = normals[i];
                glm::vec3 avgNormal = currNorm;
                int neighborCount = 1;

                int gx = (int)std::floor(currPos.x / cellSize);
                int gy = (int)std::floor(currPos.y / cellSize);
                int gz = (int)std::floor(currPos.z / cellSize);

                for (int dz = -1; dz <= 1; ++dz)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            auto it = grid.find({ gx + dx, gy + dy, gz + dz });
                            if (it == grid.end()) continue;

                            for (int neighborIdx : it->second)
                            {
                                if (i == neighborIdx) continue;

                                float distSq = glm::distance2(currPos, points[neighborIdx]);
                                if (distSq <= radius * radius)
                                {
                                    avgNormal += normals[neighborIdx];
                                    neighborCount++;
                                }
                            }
                        }
                    }
                }

                if (neighborCount > 1)
                {
                    avgNormal = glm::normalize(avgNormal);
                    float dotVal = glm::clamp(glm::dot(currNorm, avgNormal), -1.0f, 1.0f);
                    pointDivergences[i] = 1.0f - dotVal;
                }
            });

        // --- Pass 1: Mark High Divergence Points (Noise/Edge Filtering) ---
        // Mark points with high divergence as NOISE (-2) so they are ignored in Pass 2.
        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
            {
                //if (pointDivergences[i] > maxDivergence)
                if (0 > pointDivergences[i] || maxDivergence < pointDivergences[i])
                {
                    pointClusterIds[i] = -2; // Mark as Noise/High Divergence
                }
            });

        // --- Pass 2: Main Clustering (Region Growing on Smooth Points) ---
        int currentClusterId = 0;
        std::queue<int> q;

        for (int i = 0; i < (int)numPoints; ++i)
        {
            // Skip if already visited (>= 0) or marked as noise (-2)
            if (pointClusterIds[i] != -1) continue;

            // Start new cluster
            pointClusterIds[i] = currentClusterId;
            q.push(i);

            while (!q.empty())
            {
                int currIdx = q.front();
                q.pop();

                glm::vec3 currPos = points[currIdx];
                glm::vec3 currNorm = normals[currIdx];

                int gx = (int)std::floor(currPos.x / cellSize);
                int gy = (int)std::floor(currPos.y / cellSize);
                int gz = (int)std::floor(currPos.z / cellSize);

                for (int dz = -1; dz <= 1; ++dz)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            auto it = grid.find({ gx + dx, gy + dy, gz + dz });
                            if (it == grid.end()) continue;

                            for (int neighborIdx : it->second)
                            {
                                // Critical Check:
                                // If neighbor is already visited OR marked as noise (-2), skip it.
                                // This effectively removes the "different label" (noise) points.
                                if (pointClusterIds[neighborIdx] != -1) continue;

                                float distSq = glm::distance2(currPos, points[neighborIdx]);
                                if (distSq > radius * radius) continue;

                                // Normal Similarity Check
                                float dotVal = glm::dot(currNorm, normals[neighborIdx]);
                                if (dotVal < thresholdDot) continue;

                                // Add to current cluster
                                pointClusterIds[neighborIdx] = currentClusterId;
                                q.push(neighborIdx);
                            }
                        }
                    }
                }
            }
            currentClusterId++;
        }

        // Generate Colors
        clusterColors.resize(currentClusterId);
        std::srand(0);
        for (int i = 0; i < currentClusterId; ++i)
        {
            clusterColors[i] = glm::vec3((float)rand() / RAND_MAX, (float)rand() / RAND_MAX, (float)rand() / RAND_MAX);
        }

        alog("Point Clustering Done. Found %d valid clusters (Noise ignored).\n", currentClusterId);
        TE(PointClustering);
    }

    // Add this function inside struct PointCloudClusterer

    void Process_UnionAndFind(
        const std::vector<glm::vec3>& points,
        const std::vector<glm::vec3>& normals,
        float radius,
        float angleThresholdDeg)
    {
        TS(PointClustering_UnionFind);
        size_t numPoints = points.size();
        pointClusterIds.assign(numPoints, -1);

        if (points.empty() || normals.empty()) return;

        float thresholdDot = cos(glm::radians(angleThresholdDeg));
        float cellSize = radius;
        float radiusSq = radius * radius;

        // 1. Initialize Union-Find (Disjoint Set)
        // parent[i] stores the parent index of point i. 
        // If parent[i] == i, it is a root.
        std::vector<int> parent(numPoints);
        std::iota(parent.begin(), parent.end(), 0);

        // Helper Lambda: Find with Path Compression
        std::function<int(int)> findRoot = [&](int i) -> int {
            if (parent[i] == i) return i;
            return parent[i] = findRoot(parent[i]);
            };

        // Helper Lambda: Union
        auto unionSets = [&](int i, int j) {
            int rootA = findRoot(i);
            int rootB = findRoot(j);
            if (rootA != rootB)
            {
                // Simple union: assign one root to another
                // (Rank optimization could be added here for extra speed)
                parent[rootB] = rootA;
            }
            };

        // 2. Build Spatial Index (Grid)
        std::unordered_map<PointGridKey, std::vector<int>, PointGridKeyHash> grid;
        grid.reserve(numPoints);

        for (int i = 0; i < (int)numPoints; ++i)
        {
            PointGridKey key = {
                (int)std::floor(points[i].x / cellSize),
                (int)std::floor(points[i].y / cellSize),
                (int)std::floor(points[i].z / cellSize)
            };
            grid[key].push_back(i);
        }

        // 3. Process Logic: Check neighbors and Union
        // We iterate through all points and connect them with valid neighbors.
        // To avoid double checking, we can enforce i < neighborIdx or just check all.
        // Given the grid optimization, checking all close neighbors is fast enough.

        // This loop can be parallelized if we use atomic CAS for Union, 
        // but for simplicity and stability, we run it serially here.
        for (int i = 0; i < (int)numPoints; ++i)
        {
            glm::vec3 currPos = points[i];
            glm::vec3 currNorm = normals[i];

            int gx = (int)std::floor(currPos.x / cellSize);
            int gy = (int)std::floor(currPos.y / cellSize);
            int gz = (int)std::floor(currPos.z / cellSize);

            // Search 3x3x3 grid neighborhood
            for (int dz = -1; dz <= 1; ++dz)
            {
                for (int dy = -1; dy <= 1; ++dy)
                {
                    for (int dx = -1; dx <= 1; ++dx)
                    {
                        auto it = grid.find({ gx + dx, gy + dy, gz + dz });
                        if (it == grid.end()) continue;

                        for (int neighborIdx : it->second)
                        {
                            // Optimization: Process each pair only once
                            if (i >= neighborIdx) continue;

                            float distSq = glm::distance2(currPos, points[neighborIdx]);
                            if (distSq > radiusSq) continue;

                            float dotVal = glm::dot(currNorm, normals[neighborIdx]);
                            if (dotVal < thresholdDot) continue;

                            // Condition met: Union the two points
                            unionSets(i, neighborIdx);
                        }
                    }
                }
            }
        }

        // 4. Resolve Clusters (Labeling)
        // Map root indices to sequential Cluster IDs (0, 1, 2, ...)
        std::unordered_map<int, int> rootToClusterId;
        int currentClusterCount = 0;

        for (int i = 0; i < (int)numPoints; ++i)
        {
            int root = findRoot(i);

            if (rootToClusterId.find(root) == rootToClusterId.end())
            {
                rootToClusterId[root] = currentClusterCount++;
            }
            pointClusterIds[i] = rootToClusterId[root];
        }

        // 5. Generate Colors for new clusters
        clusterColors.resize(currentClusterCount);
        std::srand(0);
        for (int i = 0; i < currentClusterCount; ++i)
        {
            clusterColors[i] = glm::vec3(
                (float)rand() / RAND_MAX,
                (float)rand() / RAND_MAX,
                (float)rand() / RAND_MAX
            );
        }

        alog("Union-Find Clustering Done. Found %d clusters.\n", currentClusterCount);
        TE(PointClustering_UnionFind);
    }

    void Visualize(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals, const std::vector<glm::vec3>& colors)
    {
        if (points.empty() || pointClusterIds.empty()) return;

        size_t count = points.size();
        for (size_t i = 0; i < count; ++i)
        {
            int clusterId = pointClusterIds[i];

            // Render valid clusters
            if (clusterId >= 0 && clusterId < (int)clusterColors.size())
            {
                // Option A: Use original colors
                // auto color = colors[i];

                // Option B: Use cluster colors for debugging
                auto color = clusterColors[clusterId];

                VD::AddSphere("Points", points[i], normals[i], 0.05f, glm::vec4(color, 1.0f));
            }
            // Render Noise/High Divergence points (Optional)
            else if (clusterId == -2)
            {
                // Make noise visible as small red dots, or comment out to hide completely
                // VD::AddSphere("Points", points[i], normals[i], 0.02f, glm::vec4(1.0f, 0.0f, 0.0f, 0.5f)); 
            }
            // Unclustered valid points (Should represent error or isolated points)
            else
            {
                // VD::AddSphere("Points", points[i], normals[i], 0.05f, Color::white());
            }
        }
    }

    void VisualizeDivergenceColors(const std::vector<glm::vec3>& points, const std::vector<glm::vec3>& normals)
    {
        if (points.empty() || pointDivergences.empty()) return;

        std::vector<glm::vec3> v, n, c;
        std::vector<uint32_t> ind;
        v.reserve(points.size()); n.reserve(points.size()); c.reserve(points.size()); ind.reserve(points.size());

        for (size_t i = 0; i < points.size(); ++i)
        {
            v.push_back(points[i]);
            n.push_back(normals[i]);

            float val = glm::clamp(pointDivergences[i] * 10.0f, 0.0f, 1.0f);
            c.push_back(glm::mix(glm::vec3(0, 0, 1), glm::vec3(1, 0, 0), val));

            ind.push_back((uint32_t)v.size() - 1);
        }

        auto ent = Feather.CreateEntity("PointDivergenceDebug");
        auto rnd = Feather.CreateComponent<Renderable>(ent);
        rnd->Initialize(Renderable::GeometryMode::Points);
        rnd->AddShader(Feather.CreateShader("Default", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
        rnd->SetActiveShaderIndex(0);
        rnd->AddVertices(v); rnd->AddNormals(n); rnd->AddColors(c); rnd->AddIndices(ind);
    }

    void ToggleVisibility()
    {
        auto ent = Feather.GetEntityByName("ClusteredPoints");
        if (ent != entt::null) Feather.GetComponent<Renderable>(ent)->ToggleVisible();
    }
};

struct PointCloudCurvatureEstimator
{
    // 결과 저장: 0.0 (평면) ~ 1.0 (급격한 굴곡/엣지)
    std::vector<float> curvatures;

    struct GridKey
    {
        int x, y, z;
        bool operator==(const GridKey& o) const { return x == o.x && y == o.y && z == o.z; }
    };

    struct GridKeyHash
    {
        size_t operator()(const GridKey& k) const
        {
            return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
        }
    };

    // 3x3 대칭 행렬의 고유값 계산 (Closed-form solution for 3x3 Symmetric Matrix)
    // 반환값: vec3(lambda1, lambda2, lambda3) 정렬되지 않음
    glm::vec3 ComputeEigenValuesSymmetric(const glm::mat3& M)
    {
        double m = (M[0][0] + M[1][1] + M[2][2]) / 3.0;
        double p = (glm::pow(M[0][0] - m, 2.0) + glm::pow(M[1][1] - m, 2.0) + glm::pow(M[2][2] - m, 2.0) +
            2.0 * (glm::pow(M[0][1], 2.0) + glm::pow(M[0][2], 2.0) + glm::pow(M[1][2], 2.0))) / 6.0;

        double q = glm::determinant(M - glm::mat3(m)) / 2.0;
        double phi = 0.0;

        if (p > 1e-6)
        {
            phi = glm::atan(glm::sqrt(p * p * p - q * q), q) / 3.0;
        }

        if (phi < 0) phi += 3.14159265358979323846 / 3.0; // Correct range

        double eig1 = m + 2.0 * std::sqrt(p) * std::cos(phi);
        double eig2 = m + 2.0 * std::sqrt(p) * std::cos(phi + 2.0 * 3.14159265358979323846 / 3.0);
        double eig3 = 3.0 * m - eig1 - eig2;

        return glm::vec3((float)eig1, (float)eig2, (float)eig3);
    }

    void Process(const std::vector<glm::vec3>& points, float radius)
    {
        TS(ComputeCurvature);
        size_t numPoints = points.size();
        curvatures.assign(numPoints, 0.0f);

        if (points.empty()) return;

        float cellSize = radius;
        float radiusSq = radius * radius;

        // 1. Build Spatial Grid (이전 코드 재사용)
        std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid;
        grid.reserve(numPoints);

        for (int i = 0; i < (int)numPoints; ++i)
        {
            GridKey key = {
                (int)std::floor(points[i].x / cellSize),
                (int)std::floor(points[i].y / cellSize),
                (int)std::floor(points[i].z / cellSize)
            };
            grid[key].push_back(i);
        }

        std::vector<int> indices(numPoints);
        std::iota(indices.begin(), indices.end(), 0);

        // 2. Compute Surface Variation (Parallel)
        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
            {
                glm::vec3 currPos = points[i];

                // Collect Neighbors
                std::vector<int> neighbors;
                neighbors.reserve(32);

                int gx = (int)std::floor(currPos.x / cellSize);
                int gy = (int)std::floor(currPos.y / cellSize);
                int gz = (int)std::floor(currPos.z / cellSize);

                glm::vec3 centroid(0.0f);

                for (int dz = -1; dz <= 1; ++dz)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            auto it = grid.find({ gx + dx, gy + dy, gz + dz });
                            if (it == grid.end()) continue;

                            for (int idx : it->second)
                            {
                                if (glm::distance2(currPos, points[idx]) <= radiusSq)
                                {
                                    neighbors.push_back(idx);
                                    centroid += points[idx];
                                }
                            }
                        }
                    }
                }

                size_t k = neighbors.size();
                if (k < 4) // 점이 너무 적으면 곡률 계산 불가 (최소 4개 권장)
                {
                    curvatures[i] = 0.0f;
                    return;
                }

                // Compute Centroid
                centroid /= (float)k;

                // Compute Covariance Matrix (3x3 Symmetric)
                // Cov = Sum( (p - centroid) * (p - centroid)^T )
                float xx = 0, xy = 0, xz = 0;
                float yy = 0, yz = 0, zz = 0;

                for (int idx : neighbors)
                {
                    glm::vec3 r = points[idx] - centroid;
                    xx += r.x * r.x;
                    xy += r.x * r.y;
                    xz += r.x * r.z;
                    yy += r.y * r.y;
                    yz += r.y * r.z;
                    zz += r.z * r.z;
                }

                glm::mat3 cov;
                cov[0][0] = xx; cov[0][1] = xy; cov[0][2] = xz;
                cov[1][0] = xy; cov[1][1] = yy; cov[1][2] = yz;
                cov[2][0] = xz; cov[2][1] = yz; cov[2][2] = zz;

                cov /= (float)k;

                // Compute Eigenvalues
                glm::vec3 evals = ComputeEigenValuesSymmetric(cov);

                // Sort Eigenvalues (lambda0 <= lambda1 <= lambda2)
                // 3개라 단순 비교 정렬
                float lambda0 = evals.x, lambda1 = evals.y, lambda2 = evals.z;
                if (lambda0 > lambda1) std::swap(lambda0, lambda1);
                if (lambda1 > lambda2) std::swap(lambda1, lambda2);
                if (lambda0 > lambda1) std::swap(lambda0, lambda1);

                // Avoid division by zero
                float sum = lambda0 + lambda1 + lambda2;
                if (sum > 1e-8f)
                {
                    // Surface Variation Formula: lambda0 / (lambda0 + lambda1 + lambda2)
                    // lambda0 (최소 고유값)는 평면의 법선 방향 분산(두께)을 나타냄
                    curvatures[i] = lambda0 / sum;
                }
                else
                {
                    curvatures[i] = 0.0f;
                }
            });

        TE(ComputeCurvature);
    }

    void Visualize(const std::vector<glm::vec3>& points)
    {
        if (points.empty() || curvatures.empty()) return;

        // Heatmap 색상 (Blue -> Green -> Red)
        for (size_t i = 0; i < points.size(); ++i)
        {
            points[i];

            // 곡률 시각화를 위해 값 증폭 (보통 곡률 값은 매우 작음)
            float val = glm::clamp(curvatures[i] * 50.0f, 0.0f, 1.0f);

            // Simple Heatmap: Low(Blue) -> High(Red)
            glm::vec3 color = glm::mix(glm::vec3(0, 0, 1), glm::vec3(1, 0, 0), val);

            VD::AddSphere("Curvature", points[i], 0.05f, glm::vec4(color, 1.0f));
        }
    }
};

struct SparseDataBlock
{
    float voxelSize = Configuration.voxelSize;
    glm::vec3 gridOrigin = glm::zero<glm::vec3>();
    std::unordered_map<DataBlockKey, std::unique_ptr<DataBlock>> dataBlocks;

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

        int lx = gx % VpB;
        if (lx < 0) lx += VpB;

        int ly = gy % VpB;
        if (ly < 0) ly += VpB;

        int lz = gz % VpB;
        if (lz < 0) lz += VpB;

        return &it->second->voxels[lz * VpB * VpB + ly * VpB + lx];
    }

    void FromPointsData(const std::vector<glm::vec3>& points,
        const std::vector<glm::vec3>& normals,
        const std::vector<glm::vec3>& colors,
        const std::vector<int>& clusterIds,
        const glm::vec3& aabbMin)
    {
        gridOrigin.x = std::floor(aabbMin.x / blockSize) * blockSize;
        gridOrigin.y = std::floor(aabbMin.y / blockSize) * blockSize;
        gridOrigin.z = std::floor(aabbMin.z / blockSize) * blockSize;

        TS(Occupy);

        float truncDist = std::max(voxelSize * 4.0f, 0.15f);

        size_t numPoints = points.size();
        std::vector<size_t> indices(numPoints);
        std::iota(indices.begin(), indices.end(), 0);

        // --- Pass 1: DataBlock 할당 (Allocation) ---
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

                    int bx = centerGx / VpB;
                    int by = centerGy / VpB;
                    int bz = centerGz / VpB;

                    glm::vec3 blockMin = gridOrigin + glm::vec3((float)bx * blockSize, (float)by * blockSize, (float)bz * blockSize);
                    auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSize);
                    localKeys.push_back({ key, blockMin });

                    glm::vec3 localP = p - blockMin;

                    float margin = truncDist + voxelSize * 0.5f;

                    bool nearX_Neg = localP.x < margin;
                    bool nearX_Pos = localP.x > blockSize - margin;
                    bool nearY_Neg = localP.y < margin;
                    bool nearY_Pos = localP.y > blockSize - margin;
                    bool nearZ_Neg = localP.z < margin;
                    bool nearZ_Pos = localP.z > blockSize - margin;

                    if (nearX_Neg || nearX_Pos || nearY_Neg || nearY_Pos || nearZ_Neg || nearZ_Pos)
                    {
                        int dx_min = nearX_Neg ? -1 : 0;
                        int dx_max = nearX_Pos ? 1 : 0;
                        int dy_min = nearY_Neg ? -1 : 0;
                        int dy_max = nearY_Pos ? 1 : 0;
                        int dz_min = nearZ_Neg ? -1 : 0;
                        int dz_max = nearZ_Pos ? 1 : 0;

                        for (int dz = dz_min; dz <= dz_max; ++dz)
                        {
                            for (int dy = dy_min; dy <= dy_max; ++dy)
                            {
                                for (int dx = dx_min; dx <= dx_max; ++dx)
                                {
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

            std::sort(std::execution::par, keysToAllocate.begin(), keysToAllocate.end(),
                [](const KeyPair& a, const KeyPair& b) { return a.first < b.first; });

            auto last = std::unique(std::execution::par, keysToAllocate.begin(), keysToAllocate.end(),
                [](const KeyPair& a, const KeyPair& b) { return a.first == b.first; });

            keysToAllocate.erase(last, keysToAllocate.end());

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

        // --- Pass 2: TSDF 계산 및 데이터 채우기 (Occupation) ---
        std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
            {
                auto p = points[i];
                glm::vec3 n = (normals.empty()) ? glm::vec3(0, 1, 0) : normals[i];
                glm::vec3 c = (colors.empty()) ? glm::vec3(1, 1, 1) : colors[i];
                int cid = (clusterIds.empty()) ? -1 : clusterIds[i]; // 포인트 클러스터 ID 가져오기

                if (c.r > 1.0f) c /= 255.0f;

                glm::vec3 vecFromOrigin = p - gridOrigin;
                int centerGx = (int)std::floor(vecFromOrigin.x / voxelSize);
                int centerGy = (int)std::floor(vecFromOrigin.y / voxelSize);
                int centerGz = (int)std::floor(vecFromOrigin.z / voxelSize);

                DataBlockKey lastKey = (DataBlockKey)-1;
                DataBlock* cachedBlock = nullptr;

                for (int dz = -1; dz <= 1; ++dz)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        for (int dx = -1; dx <= 1; ++dx)
                        {
                            int gx = centerGx + dx;
                            int gy = centerGy + dy;
                            int gz = centerGz + dz;

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
                                    lastKey = key; cachedBlock = targetBlock;
                                }
                            }

                            if (targetBlock)
                            {
                                int lx = gx % VpB; if (lx < 0) lx += VpB;
                                int ly = gy % VpB; if (ly < 0) ly += VpB;
                                int lz = gz % VpB; if (lz < 0) lz += VpB;

                                float weight = 1.0f - (dist / truncDist);
                                float sdf = glm::clamp(glm::dot(voxelCenter - p, n), -truncDist, truncDist);

                                std::lock_guard<std::mutex> lock(targetBlock->blockMutex);
                                Voxel& voxel = targetBlock->voxels[lz * VpB * VpB + ly * VpB + lx];

                                if (voxel.weight <= 0.0001f)
                                {
                                    voxel.signedDistance = sdf;
                                    voxel.color = c;
                                    voxel.normal = n;
                                    voxel.weight = weight;
                                    voxel.valid = true;
                                    voxel.clusterId = cid;
                                    voxel.divergence = 0.0f; // [Added] First point has no divergence
                                }
                                else
                                {
                                    float newW = voxel.weight + weight;

                                    // [Added] Calculate Divergence
                                    // Compare current accumulated normal with new input normal
                                    // 0.0 means identical direction, 1.0 means 90 degrees or more difference
                                    glm::vec3 currentDir = glm::normalize(voxel.normal);
                                    float dotVal = glm::clamp(glm::dot(currentDir, n), -1.0f, 1.0f);
                                    float newDiv = 1.0f - dotVal;

                                    // Accumulate divergence using weighted average
                                    voxel.divergence = (voxel.divergence * voxel.weight + newDiv * weight) / newW;

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

    void Visualize()
    {
        for (const auto& pair : dataBlocks)
        {
            const auto& block = pair.second;
            glm::vec3 blockMax = block->blockMin + glm::vec3(blockSize);
            VD::AddWiredBox("Blocks", { block->blockMin, blockMax }, Color::yellow());

            for (int z = 0; z < VpB; ++z)
            {
                for (int y = 0; y < VpB; ++y)
                {
                    for (int x = 0; x < VpB; ++x)
                    {
                        const Voxel& voxel = block->voxels[z * VpB * VpB + y * VpB + x];
                        if (voxel.valid)
                        {
                            glm::vec3 vMin = block->blockMin + glm::vec3((float)x * voxelSize, (float)y * voxelSize, (float)z * voxelSize);
                            glm::vec3 vMax = vMin + glm::vec3(voxelSize);
                            VD::AddWiredBox("Voxels", { vMin, vMax }, Color::red());
                        }
                    }
                }
            }
        }
    }
};

struct Triangle { glm::vec3 v[3]; glm::vec3 n[3]; glm::vec3 c[3]; };

struct MeshGenerator
{
    std::vector<Triangle> triangles;
    std::vector<std::pair<glm::vec3, glm::vec3>> holeEdges;

    struct GridKey
    {
        int x = 0;
        int y = 0;
        int z = 0;
    
        bool operator==(const GridKey& o) const
        {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct GridKeyHash
    {
        size_t operator()(const GridKey& k) const
        {
            return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
        }
    };

    struct SNVertex
    {
        glm::vec3 pos;
        glm::vec3 normal;
        glm::vec3 color;
    };

    void Generate(SparseDataBlock& sdb)
    {
        triangles.clear();
        holeEdges.clear();
        float isoLevel = 0.0f;

        std::vector<DataBlock*> blocks;
        blocks.reserve(sdb.dataBlocks.size());
        for (auto& pair : sdb.dataBlocks) blocks.push_back(pair.second.get());

        size_t estTris = blocks.size() * 96;
        triangles.reserve(estTris);

        std::unordered_map<GridKey, SNVertex, GridKeyHash> snVertices;
        snVertices.reserve(estTris * 0.6f);
        snVertices.max_load_factor(0.7f);
        std::mutex vertexMutex;
        std::mutex triMutex;

        const glm::ivec3 corners[8] = { {0,0,0}, {1,0,0}, {1,0,1}, {0,0,1}, {0,1,0}, {1,1,0}, {1,1,1}, {0,1,1} };
        const int edgePairs[12][2] = { {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7} };

        std::for_each(std::execution::par, blocks.begin(), blocks.end(), [&](DataBlock* block)
            {
                std::vector<std::pair<GridKey, SNVertex>> localVerts;
                localVerts.reserve(64);

                glm::vec3 diff = block->blockMin - sdb.gridOrigin;
                int startGx = (int)(diff.x / sdb.voxelSize + 0.5f);
                int startGy = (int)(diff.y / sdb.voxelSize + 0.5f);
                int startGz = (int)(diff.z / sdb.voxelSize + 0.5f);

                for (int z = 0; z < VpB; ++z)
                {
                    for (int y = 0; y < VpB; ++y)
                    {
                        for (int x = 0; x < VpB; ++x)
                        {
                            int gx = startGx + x; int gy = startGy + y; int gz = startGz + z;

                            float dists[8]; glm::vec3 colors[8], normals[8];
                            int insideCount = 0; bool allValid = true;

                            for (int i = 0; i < 8; ++i)
                            {
                                const auto* v = sdb.GetVoxelByIndex(gx + corners[i].x, gy + corners[i].y, gz + corners[i].z);
                                if (!v || !v->valid)
                                {
                                    allValid = false;
                                    break;
                                }
                                dists[i] = v->signedDistance; colors[i] = v->color; normals[i] = v->normal;
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
                                SNVertex v;
                                v.pos = avgPos / (float)intersections;
                                v.color = avgColor / (float)intersections;
                                v.normal = glm::normalize(avgNormal);
                                localVerts.push_back({ {gx, gy, gz}, v });
                            }
                        }
                    }
                }
                if (!localVerts.empty())
                {
                    std::lock_guard<std::mutex> lock(vertexMutex);
                    for (const auto& kv : localVerts) snVertices[kv.first] = kv.second;
                }
            });

        std::for_each(std::execution::par, blocks.begin(), blocks.end(), [&](DataBlock* block)
            {
                std::vector<Triangle> localTris;
                localTris.reserve(128);

                glm::vec3 diff = block->blockMin - sdb.gridOrigin;
                int startGx = (int)(diff.x / sdb.voxelSize + 0.5f);
                int startGy = (int)(diff.y / sdb.voxelSize + 0.5f);
                int startGz = (int)(diff.z / sdb.voxelSize + 0.5f);

                for (int z = 0; z < VpB; ++z)
                {
                    for (int y = 0; y < VpB; ++y)
                    {
                        for (int x = 0; x < VpB; ++x)
                        {
                            int gx = startGx + x; int gy = startGy + y; int gz = startGz + z;
                            const auto* vCurr = sdb.GetVoxelByIndex(gx, gy, gz);
                            if (!vCurr || !vCurr->valid) continue;
                            bool bCurr = vCurr->signedDistance < isoLevel;

                            auto AddQuadLoc = [&](const GridKey& k1, const GridKey& k2, const GridKey& k3, const GridKey& k4, bool flip) {
                                if (snVertices.count(k1) && snVertices.count(k2) && snVertices.count(k3) && snVertices.count(k4))
                                {
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
                if (!localTris.empty())
                {
                    std::lock_guard<std::mutex> lock(triMutex);
                    triangles.insert(triangles.end(), localTris.begin(), localTris.end());
                }
            });
    }

    void Visualize(bool showMesh, bool showHoles)
    {
        if (showMesh)
        {
            std::vector<glm::vec3> v, n, c; std::vector<uint32_t> ind;
            size_t cnt = triangles.size() * 3;
            v.reserve(cnt); n.reserve(cnt); c.reserve(cnt); ind.reserve(cnt);
            uint32_t i = 0;
            for (const auto& t : triangles)
            {
                v.push_back(t.v[0]); v.push_back(t.v[1]); v.push_back(t.v[2]);
                n.push_back(t.n[0]); n.push_back(t.n[1]); n.push_back(t.n[2]);
                c.push_back(t.c[0]); c.push_back(t.c[1]); c.push_back(t.c[2]);
                ind.push_back(i++); ind.push_back(i++); ind.push_back(i++);
            }
            auto ent = Feather.CreateEntity("Mesh");
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
        if (showHoles)
        {
            for (const auto& edge : holeEdges) VD::AddLine("Holes", edge.first, edge.second, Color::red());
        }
    }

    void ExportPLY(const std::string& filename)
    {
        PLYFormat ply;
        for (const auto& t : triangles)
        {
            ply.AddPoint(t.v[0].x, t.v[0].y, t.v[0].z); ply.AddNormal(t.n[0].x, t.n[0].y, t.n[0].z); ply.AddColor(t.c[0].x, t.c[0].y, t.c[0].z);
            ply.AddPoint(t.v[1].x, t.v[1].y, t.v[1].z); ply.AddNormal(t.n[1].x, t.n[1].y, t.n[1].z); ply.AddColor(t.c[1].x, t.c[1].y, t.c[1].z);
            ply.AddPoint(t.v[2].x, t.v[2].y, t.v[2].z); ply.AddNormal(t.n[2].x, t.n[2].y, t.n[2].z); ply.AddColor(t.c[2].x, t.c[2].y, t.c[2].z);
            size_t s = ply.GetPoints().size() / 3;
            ply.AddFace((uint32_t)s - 3, (uint32_t)s - 2, (uint32_t)s - 1);
        }
        if (ply.Serialize(filename)) printf("Exported PLY: %s (Tris: %zu)\n", filename.c_str(), triangles.size());
    }

    void DetectHoles()
    {
        float tol = 0.0001f;
        std::map<std::pair<int, int>, int> edges;
        std::unordered_map<GridKey, int, GridKeyHash> vMap;
        vMap.reserve(triangles.size());
        int vCount = 0;
        std::vector<int> triIndices; triIndices.reserve(triangles.size() * 3);
        std::vector<glm::vec3> tempVerts; tempVerts.reserve(triangles.size());

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
    Feather.SetConsoleWindowIndex(3);
    Feather.SetMainWindowIndex(2);
    auto w = Feather.GetFeatherWindow();

    {
        auto appMain = Feather.CreateEntity("AppMain");
        Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
            if (GLFW_KEY_ESCAPE == event.keyCode) glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
            else if (GLFW_KEY_SPACE == event.keyCode && event.action == 0) Feather.GetImmediateModeRenderSystem()->ToggleEnable();
            else if (GLFW_KEY_BACKSPACE == event.keyCode && event.action == 0) VD::SetVisiblilityAll(false);
            else if (GLFW_KEY_F1 == event.keyCode && event.action == 0)
            {
                //VD::ToggleVisibility("Mesh");
                auto entity = Feather.GetEntityByName("Mesh");
                auto renderable = Feather.GetComponent<Renderable>(entity);
                if (renderable)
                {
                    renderable->ToggleVisible();
                }
            }
            else if (GLFW_KEY_F2 == event.keyCode && event.action == 0) VD::ToggleVisibility("Blocks");
            else if (GLFW_KEY_F3 == event.keyCode && event.action == 0) VD::ToggleVisibility("Voxels");
            else if (GLFW_KEY_F4 == event.keyCode && event.action == 0) VD::ToggleVisibility("Points");
            else if (GLFW_KEY_F5 == event.keyCode && event.action == 0) VD::ToggleVisibility("DLCs");
            else if (GLFW_KEY_F6 == event.keyCode && event.action == 0) VD::ToggleVisibility("Curvature");
            else if (GLFW_KEY_F7 == event.keyCode && event.action == 0) VD::ToggleVisibility("Holes");
            });
    }
    {
        Entity cam = Feather.CreateEntity("Camera");
        auto pcam = Feather.CreateComponent<Camera>(cam);
        auto pcamMan = Feather.CreateComponent<CameraManipulatorTrackball>(cam);
        pcamMan->SetCamera(pcam);
        Feather.CreateEventCallback<FrameBufferResizeEvent>(cam, [pcam](Entity entity, const FrameBufferResizeEvent& event) {
            pcam->GetPerspectiveSettings().SetAspectRatio((f32)Feather.GetFeatherWindow()->GetWidth() / (f32)Feather.GetFeatherWindow()->GetHeight());
            });
        Feather.CreateEventCallback<KeyEvent>(cam, [](Entity entity, const KeyEvent& event) { Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnKey(event); });
        Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity entity, const MousePositionEvent& event) { Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMousePosition(event); });
        Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity entity, const MouseButtonEvent& event) {
            auto manipulator = Feather.GetComponent<CameraManipulatorTrackball>(entity);
            manipulator->OnMouseButton(event);
            });
        Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) { Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event); });
    }

    typedef enum {
        DL_TOOTH,
        DL_GINGIVA1,
        DL_GINGIVA2,
        DL_TONGUE,
        DL_CHEEK,
        DL_LIP,
        DL_ETC,
        DL_DENTIFORM_TOOTH,
        DL_DENTIFORM_GINGIVA1,
        DL_DENTIFORM_GINGIVA2,
        DL_PLASTER,
        DL_FINGER,
        DL_METAL,
        DL_PALATAL,
        DL_ABUTMENT,
        DL_SCANBODY,
        DL_GINGIVA3,	//	mscho	@20241017
        DL_OBTURA,
        DL_3DPRTMODEL,
        DL_RETRACTOR,
        DL_CLASS_LAST
    } DL_Class_Names;

    Feather.AddOnInitializeCallback([&]()
        {
            TS(PLYLoading);
            PLYFormat ply;
            //if (!ply.Deserialize("D:\\Debug\\PLY\\InputCompound.ply"))
            if (!ply.Deserialize("D:\\Debug\\PLY\\Compound_0.ply"))
            {
                printf("Failed to load PLY.\n");
                return;
            }

            std::vector<glm::vec3> points, normals, colors;
            size_t rawCount = ply.GetPoints().size() / 3;
            points.reserve(rawCount);
            if (!ply.GetNormals().empty()) normals.reserve(rawCount);
            if (!ply.GetColors().empty()) colors.reserve(rawCount);

            auto& rawPts = ply.GetPoints();
            auto& rawNorms = ply.GetNormals();
            auto& rawCols = ply.GetColors();
            auto& rawDLCs = ply.GetDeepLearningClasses();

            bool hasN = !rawNorms.empty();
            bool hasC = !rawCols.empty();
            bool useAlpha = ply.UseAlpha();
			bool hasDLC = !rawDLCs.empty();

            auto contrastingColos = Color::GetContrastingColors(16);

            auto IsTooth = [](int deepLearningClass) -> bool
                {
                    switch (deepLearningClass)
                    {
                    case DL_TOOTH:
                        return true;
                    case DL_GINGIVA1:
                        return false;
                    case DL_GINGIVA2:
                        return false;
                    case DL_TONGUE:
                        return false;
                    case DL_CHEEK:
                        return false;
                    case DL_LIP:
                        return false;
                    case DL_ETC:
                        return false;
                    case DL_DENTIFORM_TOOTH:
                        return true;
                    case DL_DENTIFORM_GINGIVA1:
                        return false;
                    case DL_DENTIFORM_GINGIVA2:
                        return false;
                    case DL_PLASTER:
                        return false;
                    case DL_FINGER:
                        return false;
                    case DL_METAL:
                        return true;
                    case DL_PALATAL:
                        return false;
                    case DL_ABUTMENT:
                        return true;
                    case DL_SCANBODY:
                        return true;
                    case DL_GINGIVA3:
                        return false;
                    case DL_OBTURA:
                        return false;
                    case DL_3DPRTMODEL:
                        return false;
                    case DL_RETRACTOR:
                        return false;
                    case DL_CLASS_LAST:
                        return false;

                    default:
                        return false;
                        break;
                    }
                };

            for (size_t i = 0; i < rawCount; ++i)
            {
                float x = rawPts[i * 3], y = rawPts[i * 3 + 1], z = rawPts[i * 3 + 2];
                if (x >= Configuration.filterMin.x && x <= Configuration.filterMax.x &&
                    y >= Configuration.filterMin.y && y <= Configuration.filterMax.y &&
                    z >= Configuration.filterMin.z && z <= Configuration.filterMax.z)
                {
                    auto dlc = rawDLCs[i];
                    //if (DL_ETC == dlc) continue;

                    points.push_back({ x,y,z });
                    if (hasN) normals.push_back({ rawNorms[i * 3], rawNorms[i * 3 + 1], rawNorms[i * 3 + 2] });
                    if (hasC)
                    {
                        if (useAlpha) colors.push_back({ rawCols[i * 4], rawCols[i * 4 + 1], rawCols[i * 4 + 2] });
                        else colors.push_back({ rawCols[i * 3], rawCols[i * 3 + 1], rawCols[i * 3 + 2] });
                    }
                    if (hasDLC)
                    {
                        if (IsTooth(dlc))
                        {
                            VD::AddSphere("DLCs", { x,y,z }, 0.05f, Color::white());
                        }
                        else
                        {
                            if (DL_ETC == dlc)
                            {
                                VD::AddSphere("DLCs", { x,y,z }, 0.05f, Color::black());
                            }
                            else
                            {
                                auto color = contrastingColos[dlc % 16];
                                VD::AddSphere("DLCs", { x,y,z }, 0.05f, color);
                            }
                        }
                    }
                }
            }
            TE(PLYLoading);

            auto [minx, miny, minz] = ply.GetAABBMin();
            glm::vec3 aabbMin(minx - 1.0f, miny - 1.0f, minz - 1.0f);

            PointCloudCurvatureEstimator curvatureEstimator;
            curvatureEstimator.Process(points, 0.5f);
            curvatureEstimator.Visualize(points);

            PointCloudClusterer pcClusterer;
            
            pcClusterer.Process(points, normals, Configuration.voxelSize * 1.0f, 30.0f, 0.05f);
            //pcClusterer.Process_UnionAndFind(points, normals, Configuration.voxelSize * 1.3f, 15.0f);
            
            pcClusterer.Visualize(points, normals, colors);

            SparseDataBlock sdb;
            sdb.FromPointsData(points, normals, colors, pcClusterer.pointClusterIds, aabbMin);

            TS(MeshGeneration);
            static MeshGenerator meshGen;
            meshGen.Generate(sdb);
            meshGen.DetectHoles();
            TE(MeshGeneration);

            meshGen.Visualize(true, true);
            //meshGen.ExportPLY("D:\\Debug\\PLY\\output.ply");

            sdb.Visualize();

            alog("Total DataBlocks : %s\n", FormatWithCommas(sdb.dataBlocks.size()).c_str());
            alog("DataBlock Size : %zd\n", sizeof(DataBlock));
            alog("Total Memory : %s bytes\n", FormatWithCommas(sdb.dataBlocks.size() * sizeof(DataBlock)).c_str());

            {
                size_t blockCount = sdb.dataBlocks.size();
                size_t blockSizeBytes = sizeof(DataBlock);
                size_t totalBytes = blockCount * blockSizeBytes;

                auto formatBytes = [&](size_t bytes) {
                    double kb = bytes / 1024.0;
                    double mb = bytes / (1024.0 * 1024.0);
                    double gb = bytes / (1024.0 * 1024.0 * 1024.0);

                    char buf[256];
                    if (gb >= 1.0)
                        sprintf(buf, "%.2f GB", gb);
                    else if (mb >= 1.0)
                        sprintf(buf, "%.2f MB", mb);
                    else if (kb >= 1.0)
                        sprintf(buf, "%.2f KB", kb);
                    else
                        sprintf(buf, "%zu bytes", bytes);
                    return std::string(buf);
                    };

                alog("========================================\n");
                alog("SparseDataBlock Memory Report\n");
                alog("  Total Blocks : %s\n", FormatWithCommas(blockCount).c_str());
                alog("  Block Size   : %s (%zu bytes)\n",
                    formatBytes(blockSizeBytes).c_str(), blockSizeBytes);
                alog("  Total Memory : %s\n",
                    formatBytes(totalBytes).c_str());
                alog("========================================\n");
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
