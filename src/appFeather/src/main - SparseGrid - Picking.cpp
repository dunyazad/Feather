#include <robin_hood.h>

#include <libFeather.h>


using VD = VisualDebugging;

namespace GeometricProcessingPipeline
{
    class Configuration
    {
    public:
        static constexpr int voxelsPerBlockAxis = 8;
        static constexpr int voxelsPerBlock =
            voxelsPerBlockAxis * voxelsPerBlockAxis * voxelsPerBlockAxis;

        static constexpr float voxelSize = 0.1f;
        static constexpr int sdfOffset = 1;

        //static constexpr glm::vec3 filterMin = glm::vec3(-10.0f, -10.0f, -10.0f);
        //static constexpr glm::vec3 filterMax = glm::vec3(10.0f, 10.0f, 10.0f);
        inline static glm::vec3 filterMin = glm::vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        inline static glm::vec3 filterMax = glm::vec3(FLT_MAX, FLT_MAX, FLT_MAX);

        static constexpr float pointVisualizationRadius = 0.025f;
    };

    class Triangle
    {
    public:
        glm::vec3 v[3];
        glm::vec3 n[3];
        glm::vec3 c[3];
    };

    class PointCloud
    {
    public:
        size_t numberOfElements = 0;
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> normals;
		std::vector<glm::vec3> colors;

        AABB aabb;

        void FromPLY(const std::string& plyFileName)
        {
			PLYFormat ply;
			ply.Deserialize(plyFileName);
            FromPLY(ply);
        }

        void FromPLY(const PLYFormat& ply)
        {
            numberOfElements = ply.GetPoints().size() / 3;
            positions.resize(numberOfElements);
            normals.resize(numberOfElements);
            colors.resize(numberOfElements);

            memcpy(positions.data(), ply.GetPoints().data(), sizeof(float) * 3 * numberOfElements);
            if (ply.GetNormals().size() == numberOfElements * 3)
            {
                memcpy(normals.data(), ply.GetNormals().data(), sizeof(float) * 3 * numberOfElements);
            }
            if (ply.GetColors().size() == numberOfElements * 3)
            {
                memcpy(colors.data(), ply.GetColors().data(), sizeof(float) * 3 * numberOfElements);
            }
            else if (ply.GetColors().size() == numberOfElements * 4)
            {
                for (size_t i = 0; i < numberOfElements; ++i)
                {
                    colors[i] = glm::vec3(
                        ply.GetColors()[4 * i + 0],
                        ply.GetColors()[4 * i + 1],
                        ply.GetColors()[4 * i + 2]);
                }
            }

            for (size_t i = 0; i < numberOfElements; i++)
            {
				aabb.Expand(positions[i]);
            }
        }

        void ToPLY(PLYFormat& ply) const
        {
            for (size_t i = 0; i < numberOfElements; i++)
            {
                ply.AddPoint(XYZ(positions[i]));
                if (normals.size() == numberOfElements)
                {
                    ply.AddNormal(XYZ(normals[i]));
				}
                if (colors.size() == numberOfElements)
                {
                    ply.AddColor(XYZ(colors[i]));
				}
            }
		}
    };

    class Voxel
    {
    public:
        bool valid = false;
        float signedDistance = 0.0f;
        float weight = 0.0f;
        glm::vec3 normal = glm::zero<glm::vec3>();
        glm::vec3 color = glm::zero<glm::vec3>();
        int clusterId = -1;
        float divergence = 0.0f;
    };

    struct SparseGridPickResult
    {
        bool hasHit = false;    // 충돌 여부
        int pointIndex = -1;    // 가장 가까운 점의 인덱스
        uint64_t cellKey = 0;   // 해당 셀의 해시 키
        int gx = 0, gy = 0, gz = 0; // 해당 셀의 3차원 그리드 인덱스
        float distance = 0.0f;  // 레이 원점으로부터의 거리
    };

    class SparseGrid
    {
    public:
        robin_hood::unordered_flat_map<uint64_t, int> gridHead;
        std::vector<int> nextPoint;

		AABB aabb;
        float cellSize = 0.1f;

        inline uint64_t GetKey(int x, int y, int z) const
        {
            return ((uint64_t)x << 42) | ((uint64_t)y << 21) | (uint64_t)z;
        }

        void Build(const GeometricProcessingPipeline::PointCloud& pc, float cellSize)
        {
            if (pc.numberOfElements == 0) return;

            this->cellSize = cellSize;
            
            aabb = pc.aabb;

            gridHead.clear();
            gridHead.reserve(pc.numberOfElements);

            nextPoint.assign(pc.numberOfElements, -1);

            aabb.min -= glm::vec3(cellSize * 0.1f);
            aabb.max += glm::vec3(cellSize * 0.1f);

            for (int i = 0; i < (int)pc.numberOfElements; ++i)
            {
                int gx = (int)((pc.positions[i].x - aabb.min.x) / cellSize);
                int gy = (int)((pc.positions[i].y - aabb.min.y) / cellSize);
                int gz = (int)((pc.positions[i].z - aabb.min.z) / cellSize);

                uint64_t key = GetKey(gx, gy, gz);

                auto it = gridHead.find(key);

                if (it != gridHead.end())
                {
                    nextPoint[i] = it->second;
                    it->second = i;
                }
                else
                {
                    gridHead[key] = i;
                }
            }
        }

        SparseGridPickResult Pick(const std::vector<glm::vec3>& points, const Ray& ray, float pickRadius)
        {
            SparseGridPickResult result;
            if (points.empty() || gridHead.empty()) return result;

            // 1. 그리드 전체 영역과 레이 충돌 검사 (진입점 찾기)
            float tEntry = 0.0f, tExit = 0.0f;
            if (!aabb.IntersectRay(ray, tEntry, tExit)) return result; // 그리드 안 지남

            if (tEntry < 0.0f) tEntry = 0.0f; // 카메라가 그리드 안에 있음

            // 2. 진입점에서의 Grid Index 계산
            // 약간 안쪽으로 이동(epsilon)하여 경계선 오차 방지
            glm::vec3 startPos = ray.origin + ray.direction * (tEntry + 0.001f);

            // 현재 검사 중인 복셀 인덱스
            int curGx = (int)std::floor((startPos.x - aabb.min.x) / cellSize);
            int curGy = (int)std::floor((startPos.y - aabb.min.y) / cellSize);
            int curGz = (int)std::floor((startPos.z - aabb.min.z) / cellSize);

            // DDA 스텝 설정
            int stepX = (ray.direction.x > 0) ? 1 : -1;
            int stepY = (ray.direction.y > 0) ? 1 : -1;
            int stepZ = (ray.direction.z > 0) ? 1 : -1;

            // tMax: 다음 복셀 경계까지의 *절대 거리(ray.origin 기준)*
            // 다음 경계 좌표 계산
            float nextBoundX = aabb.min.x + (curGx + (stepX > 0 ? 1 : 0)) * cellSize;
            float nextBoundY = aabb.min.y + (curGy + (stepY > 0 ? 1 : 0)) * cellSize;
            float nextBoundZ = aabb.min.z + (curGz + (stepZ > 0 ? 1 : 0)) * cellSize;

            // tMax 초기값 설정 (Ray Origin 기준)
            float tMaxX = (nextBoundX - ray.origin.x) * ray.inverseDirection.x;
            float tMaxY = (nextBoundY - ray.origin.y) * ray.inverseDirection.y;
            float tMaxZ = (nextBoundZ - ray.origin.z) * ray.inverseDirection.z;

            // tDelta: 한 복셀 이동 시 증가하는 t 값 (항상 양수)
            float tDeltaX = std::abs(cellSize * ray.inverseDirection.x);
            float tDeltaY = std::abs(cellSize * ray.inverseDirection.y);
            float tDeltaZ = std::abs(cellSize * ray.inverseDirection.z);

            // 3. Grid Traversal Loop
            // tExit을 약간 넘어서까지 검사 (마지막 복셀 누락 방지)
            while (tEntry <= tExit + cellSize * 0.1f)
            {
                uint64_t key = GetKey(curGx, curGy, curGz);
                auto it = gridHead.find(key);

                if (it != gridHead.end())
                {
                    int currIdx = it->second;
                    float minTInVoxel = FLT_MAX;
                    bool hitFound = false;

                    // 현재 복셀의 점들 검사
                    while (currIdx != -1)
                    {
                        float t = 0.0f;
                        if (ray.IntersectSphere(points[currIdx], pickRadius, t))
                        {
                            // 가장 가까운 점 찾기
                            if (t < minTInVoxel)
                            {
                                minTInVoxel = t;
                                result.hasHit = true;
                                result.pointIndex = currIdx;
                                result.distance = t;
                                result.gx = curGx; result.gy = curGy; result.gz = curGz;
                                hitFound = true;
                            }
                        }
                        currIdx = nextPoint[currIdx];
                    }

                    // [핵심] 현재 복셀에서 충돌을 찾았고, 그 충돌이 '이 복셀의 범위 안'에 있다면 종료
                    // 즉, 현재 복셀을 벗어나기 전에 충돌했다면 그게 제일 가까운 거임.
                    // 현재 복셀을 벗어나는 시점(t_next_boundary) 계산
                    float tNextBoundary = std::min(std::min(tMaxX, tMaxY), tMaxZ);

                    if (hitFound && minTInVoxel <= tNextBoundary + 0.01f) // Margin 약간 추가
                    {
                        return result;
                    }
                }

                // 다음 복셀로 이동
                if (tMaxX < tMaxY) {
                    if (tMaxX < tMaxZ) {
                        curGx += stepX;
                        tEntry = tMaxX; // 현재 t 업데이트
                        tMaxX += tDeltaX;
                    }
                    else {
                        curGz += stepZ;
                        tEntry = tMaxZ;
                        tMaxZ += tDeltaZ;
                    }
                }
                else {
                    if (tMaxY < tMaxZ) {
                        curGy += stepY;
                        tEntry = tMaxY;
                        tMaxY += tDeltaY;
                    }
                    else {
                        curGz += stepZ;
                        tEntry = tMaxZ;
                        tMaxZ += tDeltaZ;
                    }
                }
            }

            return result;
        }

        // 단순 무식한 전체 검사 (디버깅용)
        SparseGridPickResult PickBruteForce(const std::vector<glm::vec3>& points, const Ray& ray, float pickRadius)
        {
            SparseGridPickResult result;
            float minT = FLT_MAX;
            for (size_t i = 0; i < points.size(); ++i) {
                float t;
                if (ray.IntersectSphere(points[i], pickRadius, t)) {
                    if (t < minT) {
                        minT = t;
                        result.hasHit = true;
                        result.pointIndex = (int)i;
                        result.distance = t;
                    }
                }
            }
            return result;
        }

        void Visualize(const GeometricProcessingPipeline::PointCloud& pc)
        {
            const uint64_t mask = 0x1FFFFF;

            for (const auto& pair : gridHead)
            {
                uint64_t key = pair.first;
                int headIdx = pair.second;

                uint64_t gz = key & mask;
                uint64_t gy = (key >> 21) & mask;
                uint64_t gx = (key >> 42);

                glm::vec3 cellMin = aabb.min + glm::vec3((float)gx * cellSize, (float)gy * cellSize, (float)gz * cellSize);
                glm::vec3 cellMax = cellMin + glm::vec3(cellSize);

                VD::AddWiredBox("SparseGridCells", { cellMin, cellMax }, glm::vec4(0.0f, 1.0f, 0.0f, 0.3f));

                int currIdx = headIdx;
                while (currIdx != -1)
                {
                    const glm::vec3& p = pc.positions[currIdx];
					const glm::vec3& n = glm::normalize(pc.normals[currIdx]);
					const glm::vec3& c = pc.colors[currIdx];

                    VD::AddSphere("SparseGridPoints", p, n, GeometricProcessingPipeline::Configuration::pointVisualizationRadius, glm::vec4(c, 1.0f));

                    currIdx = nextPoint[currIdx];
                }
            }
        }
    };

    typedef uint64_t DataBlockKey;

    class DataBlock
    {
    public:
        glm::vec3 blockMin = glm::zero<glm::vec3>();
        Voxel voxels[Configuration::voxelsPerBlock];
        std::mutex blockMutex;

        void Initialize()
        {
            //memset(voxels, 0, sizeof(Voxel) * Configuration::voxelsPerBlock);

            for (int i = 0; i < Configuration::voxelsPerBlock; ++i)
            {
                voxels[i] = Voxel();
            }
        }
    };

    class IGeometricProcessingOperator
    {
    public:
        virtual void Process(PointCloud& pointCloud) = 0;
		virtual void Visualize() = 0;
	};

    class OperatorClustering : public IGeometricProcessingOperator
    {
    public:
        virtual void Process(PointCloud& pointCloud) override
        {
        }
        virtual void Visualize() override
        {
        }
    };
    

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
                    // VD::AddSphere("Points", points[i], normals[i], 0.02f, glm::vec4(1.0f, 0.0f, 0.0f, Configuration::pointVisualizationRadius)); 
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
                // 곡률 시각화를 위해 값 증폭 (보통 곡률 값은 매우 작음)
                float val = glm::clamp(curvatures[i] * 10.0f, 0.0f, 1.0f);

                // Simple Heatmap: Low(Blue) -> High(Red)
                glm::vec3 color = glm::mix(glm::vec3(0, 0, 1), glm::vec3(1, 0, 0), val);

                if (0.5f > curvatures[i] * 10.0f)
                {
                    VD::AddSphere("Curvature", points[i], 0.05f, glm::vec4(color, 1.0f));
                }
            }
        }
    };

    struct SparseDataBlock
    {
        float voxelSize = Configuration::voxelSize;
        glm::vec3 gridOrigin = glm::zero<glm::vec3>();
        std::unordered_map<DataBlockKey, std::unique_ptr<DataBlock>> dataBlocks;

        float blockSizePerAxis = voxelSize * Configuration::voxelsPerBlockAxis;

        Voxel* GetVoxelByIndex(int gx, int gy, int gz)
        {
            int bx = (int)floor((float)gx / Configuration::voxelsPerBlockAxis);
            int by = (int)floor((float)gy / Configuration::voxelsPerBlockAxis);
            int bz = (int)floor((float)gz / Configuration::voxelsPerBlockAxis);

            glm::vec3 blockMin = gridOrigin + glm::vec3((float)bx * blockSizePerAxis, (float)by * blockSizePerAxis, (float)bz * blockSizePerAxis);
            auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSizePerAxis);

            auto it = dataBlocks.find(key);
            if (it == dataBlocks.end()) return nullptr;

            int lx = gx % Configuration::voxelsPerBlockAxis;
            if (lx < 0) lx += Configuration::voxelsPerBlockAxis;

            int ly = gy % Configuration::voxelsPerBlockAxis;
            if (ly < 0) ly += Configuration::voxelsPerBlockAxis;

            int lz = gz % Configuration::voxelsPerBlockAxis;
            if (lz < 0) lz += Configuration::voxelsPerBlockAxis;

            return &it->second->voxels[lz * Configuration::voxelsPerBlockAxis * Configuration::voxelsPerBlockAxis + ly * Configuration::voxelsPerBlockAxis + lx];
        }

        void FromPointsData(const std::vector<glm::vec3>& points,
            const std::vector<glm::vec3>& normals,
            const std::vector<glm::vec3>& colors,
            const std::vector<int>& clusterIds,
            const glm::vec3& aabbMin)
        {
            gridOrigin.x = std::floor(aabbMin.x / blockSizePerAxis) * blockSizePerAxis;
            gridOrigin.y = std::floor(aabbMin.y / blockSizePerAxis) * blockSizePerAxis;
            gridOrigin.z = std::floor(aabbMin.z / blockSizePerAxis) * blockSizePerAxis;

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

                        int bx = centerGx / Configuration::voxelsPerBlockAxis;
                        int by = centerGy / Configuration::voxelsPerBlockAxis;
                        int bz = centerGz / Configuration::voxelsPerBlockAxis;

                        glm::vec3 blockMin = gridOrigin + glm::vec3((float)bx * blockSizePerAxis, (float)by * blockSizePerAxis, (float)bz * blockSizePerAxis);
                        auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSizePerAxis);
                        localKeys.push_back({ key, blockMin });

                        glm::vec3 localP = p - blockMin;

                        float margin = truncDist + voxelSize * 0.5f;

                        bool nearX_Neg = localP.x < margin;
                        bool nearX_Pos = localP.x > blockSizePerAxis - margin;
                        bool nearY_Neg = localP.y < margin;
                        bool nearY_Pos = localP.y > blockSizePerAxis - margin;
                        bool nearZ_Neg = localP.z < margin;
                        bool nearZ_Pos = localP.z > blockSizePerAxis - margin;

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

                                        glm::vec3 nbMin = gridOrigin + glm::vec3((float)(bx + dx) * blockSizePerAxis, (float)(by + dy) * blockSizePerAxis, (float)(bz + dz) * blockSizePerAxis);
                                        auto nKey = Morton3D::EncodeFromVec3(nbMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSizePerAxis);
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

                                int bx = (int)floor((float)gx / Configuration::voxelsPerBlockAxis);
                                int by = (int)floor((float)gy / Configuration::voxelsPerBlockAxis);
                                int bz = (int)floor((float)gz / Configuration::voxelsPerBlockAxis);

                                float currBlockSize = blockSizePerAxis;
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
                                    int lx = gx % Configuration::voxelsPerBlockAxis; if (lx < 0) lx += Configuration::voxelsPerBlockAxis;
                                    int ly = gy % Configuration::voxelsPerBlockAxis; if (ly < 0) ly += Configuration::voxelsPerBlockAxis;
                                    int lz = gz % Configuration::voxelsPerBlockAxis; if (lz < 0) lz += Configuration::voxelsPerBlockAxis;

                                    float weight = 1.0f - (dist / truncDist);
                                    float sdf = glm::clamp(glm::dot(voxelCenter - p, n), -truncDist, truncDist);

                                    std::lock_guard<std::mutex> lock(targetBlock->blockMutex);
                                    Voxel& voxel = targetBlock->voxels[lz * Configuration::voxelsPerBlockAxis * Configuration::voxelsPerBlockAxis + ly * Configuration::voxelsPerBlockAxis + lx];

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
                glm::vec3 blockMax = block->blockMin + glm::vec3(blockSizePerAxis);
                VD::AddWiredBox("Blocks", { block->blockMin, blockMax }, Color::yellow());

                for (int z = 0; z < Configuration::voxelsPerBlockAxis; ++z)
                {
                    for (int y = 0; y < Configuration::voxelsPerBlockAxis; ++y)
                    {
                        for (int x = 0; x < Configuration::voxelsPerBlockAxis; ++x)
                        {
                            const Voxel& voxel = block->voxels[z * Configuration::voxelsPerBlockAxis * Configuration::voxelsPerBlockAxis + y * Configuration::voxelsPerBlockAxis + x];
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

                    for (int z = 0; z < Configuration::voxelsPerBlockAxis; ++z)
                    {
                        for (int y = 0; y < Configuration::voxelsPerBlockAxis; ++y)
                        {
                            for (int x = 0; x < Configuration::voxelsPerBlockAxis; ++x)
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

                    for (int z = 0; z < Configuration::voxelsPerBlockAxis; ++z)
                    {
                        for (int y = 0; y < Configuration::voxelsPerBlockAxis; ++y)
                        {
                            for (int x = 0; x < Configuration::voxelsPerBlockAxis; ++x)
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
}

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

    GeometricProcessingPipeline::PointCloud pc;
    GeometricProcessingPipeline::SparseGrid sgrid;


    {
        auto appMain = Feather.CreateEntity("AppMain");
        Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
            if (GLFW_KEY_ESCAPE == event.keyCode) glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
            else if (GLFW_KEY_SPACE == event.keyCode && event.action == 0) Feather.GetImmediateModeRenderSystem()->ToggleEnable();
            else if (GLFW_KEY_BACKSPACE == event.keyCode && event.action == 0) VD::SetVisibilityAll(false);

#if 0
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
#endif // 0
            else if (GLFW_KEY_F1 == event.keyCode && event.action == 0) VD::ToggleVisibility("SparseGridPoints");
            else if (GLFW_KEY_F2 == event.keyCode && event.action == 0) VD::ToggleVisibility("SparseGridCells");
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

            if (event.button == GLFW_MOUSE_BUTTON_LEFT && event.action == 1) // Press
            {
                VD::Clear("PickedPoint");
                VD::Clear("PickedCell");

                auto window = Feather.GetFeatherWindow();
                GLFWwindow* nativeWin = window->GetGLFWwindow();

                // [Fix 3] DPI Awareness (High-DPI Display Support)
                int winW, winH, fbW, fbH;
                glfwGetWindowSize(nativeWin, &winW, &winH);
                glfwGetFramebufferSize(nativeWin, &fbW, &fbH);

                double mx, my;
                glfwGetCursorPos(nativeWin, &mx, &my);

                // 마우스 좌표 보정
                float pxRatio = (float)fbW / (float)winW;
                float pyRatio = (float)fbH / (float)winH;
                mx *= pxRatio;
                my *= pyRatio;

                // [Fix 4] Robust Ray Generation (Inverse ViewProj)
                // NDC 좌표 (-1 ~ 1)
                float ndcX = (2.0f * (float)mx) / (float)fbW - 1.0f;
                float ndcY = 1.0f - (2.0f * (float)my) / (float)fbH;

                auto cameraComp = Feather.GetComponent<Camera>(cam);
                glm::mat4 view = cameraComp->GetViewMatrix();
                glm::mat4 proj = cameraComp->GetProjectionMatrix();
                glm::mat4 invVP = glm::inverse(proj * view);

                // Unproject Near/Far points
                glm::vec4 screenPosNear(ndcX, ndcY, -1.0f, 1.0f);
                glm::vec4 screenPosFar(ndcX, ndcY, 1.0f, 1.0f);

                glm::vec4 worldPosNear = invVP * screenPosNear;
                glm::vec4 worldPosFar = invVP * screenPosFar;

                // Perspective Divide
                if (worldPosNear.w != 0.0f) worldPosNear /= worldPosNear.w;
                if (worldPosFar.w != 0.0f) worldPosFar /= worldPosFar.w;

                glm::vec3 rayOrigin = glm::vec3(worldPosNear);
                glm::vec3 rayDir = glm::normalize(glm::vec3(worldPosFar - worldPosNear));

                Ray ray{ rayOrigin, rayDir };

                // Picking 실행
                // 0.15f는 선택 반경입니다. 점 크기에 맞춰 조절하세요.
                auto result = sgrid.Pick(pc.positions, ray, GeometricProcessingPipeline::Configuration::pointVisualizationRadius);

                if (result.hasHit)
                {
                    // 점 시각화 (빨간색)
                    glm::vec3 p = pc.positions[result.pointIndex];
                    VD::AddSphere("PickedPoint", p, GeometricProcessingPipeline::Configuration::pointVisualizationRadius * 1.1f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

					manipulator->SetCenter(p);

                    // 셀 시각화 (파란색 박스)
                    glm::vec3 cellMin = sgrid.aabb.min + glm::vec3(
                        (float)result.gx * sgrid.cellSize,
                        (float)result.gy * sgrid.cellSize,
                        (float)result.gz * sgrid.cellSize
                    );
                    glm::vec3 cellMax = cellMin + glm::vec3(sgrid.cellSize);
                    VD::AddWiredBox("PickedCell", { cellMin, cellMax }, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));

                    alog("Hit! Idx:%d, Cell(%d,%d,%d), Dist:%.2f\n", result.pointIndex, result.gx, result.gy, result.gz, result.distance);
                }
            }
            });
        Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) { Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event); });
    }

    {
		auto entity = Feather.CreateEntity("TreeViewPanel");
		auto component = Feather.CreateComponent<TreeViewPanel>(entity);
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
            {
                TS(PLYLoading);
				PLYFormat ply;
                if (!ply.Deserialize("D:\\Debug\\PLY\\Input.ply"))
                {
                    printf("Failed to load PLY.\n");
                    return;
                }
                pc.FromPLY(ply);
                TE(PLYLoading);
            }

            {
                TS(SparseGridBuilding);
                sgrid.Build(pc, 0.1f);
                TE(SparseGridBuilding);
			}

            sgrid.Visualize(pc);
            return;




















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
                if (x >= GeometricProcessingPipeline::Configuration::filterMin.x && x <= GeometricProcessingPipeline::Configuration::filterMax.x &&
                    y >= GeometricProcessingPipeline::Configuration::filterMin.y && y <= GeometricProcessingPipeline::Configuration::filterMax.y &&
                    z >= GeometricProcessingPipeline::Configuration::filterMin.z && z <= GeometricProcessingPipeline::Configuration::filterMax.z)
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

            GeometricProcessingPipeline::PointCloudCurvatureEstimator curvatureEstimator;
            curvatureEstimator.Process(points, 0.5f);
            curvatureEstimator.Visualize(points);

            GeometricProcessingPipeline::PointCloudClusterer pcClusterer;
            
            pcClusterer.Process(points, normals, GeometricProcessingPipeline::Configuration::voxelSize * 1.0f, 30.0f, 0.05f);
            //pcClusterer.Process_UnionAndFind(points, normals, Configuration::voxelSize * 1.3f, 15.0f);
            
            pcClusterer.Visualize(points, normals, colors);

            GeometricProcessingPipeline::SparseDataBlock sdb;
            sdb.FromPointsData(points, normals, colors, pcClusterer.pointClusterIds, aabbMin);

            TS(MeshGeneration);
            GeometricProcessingPipeline::MeshGenerator meshGen;
            meshGen.Generate(sdb);
            meshGen.DetectHoles();
            TE(MeshGeneration);

            meshGen.Visualize(true, true);
            //meshGen.ExportPLY("D:\\Debug\\PLY\\output.ply");

            sdb.Visualize();

            alog("Total DataBlocks : %s\n", FormatWithCommas(sdb.dataBlocks.size()).c_str());
            alog("DataBlock Size : %zd\n", sizeof(GeometricProcessingPipeline::DataBlock));
            alog("Total Memory : %s bytes\n", FormatWithCommas(sdb.dataBlocks.size() * sizeof(GeometricProcessingPipeline::DataBlock)).c_str());

            {
                size_t blockCount = sdb.dataBlocks.size();
                size_t blockSizeBytes = sizeof(GeometricProcessingPipeline::DataBlock);
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
