#include <robin_hood.h>

#include <libFeather.h>


static inline std::string FormatWithCommas(size_t value)
{
    std::string numStr = std::to_string(value);
    int insertPosition = static_cast<int>(numStr.length()) - 3;
    while (insertPosition > 0) { numStr.insert(insertPosition, ","); insertPosition -= 3; }
    return numStr;
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

static inline bool IsTooth(int deepLearningClass)
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
}

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

        //static constexpr float pointVisualizationRadius = 0.045f;
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
        std::vector<int> pointClassIDs;
        std::vector<int> marks;

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
			pointClassIDs.resize(numberOfElements, -1);
			marks.resize(numberOfElements, -1);

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
            if(ply.GetDeepLearningClasses().size() == numberOfElements)
            {
                memcpy(pointClassIDs.data(), ply.GetDeepLearningClasses().data(), sizeof(int) * numberOfElements);
			}

            for (size_t i = 0; i < numberOfElements; i++)
            {
				aabb.Expand(positions[i]);
            }
        }

        void ToPLY(const std::string& plyFileName) const
        {
            PLYFormat ply;
            ToPLY(ply);
			ply.Serialize(plyFileName);
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

    class ISpatialPartitioning {};

	class SparseGrid : public ISpatialPartitioning
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

            float tEntry = 0.0f, tExit = 0.0f;
            if (!aabb.IntersectRay(ray, tEntry, tExit)) return result;

            if (tEntry < 0.0f) tEntry = 0.0f;

            glm::vec3 startPos = ray.origin + ray.direction * (tEntry + 0.001f);

            int curGx = (int)std::floor((startPos.x - aabb.min.x) / cellSize);
            int curGy = (int)std::floor((startPos.y - aabb.min.y) / cellSize);
            int curGz = (int)std::floor((startPos.z - aabb.min.z) / cellSize);

            int stepX = (ray.direction.x > 0) ? 1 : -1;
            int stepY = (ray.direction.y > 0) ? 1 : -1;
            int stepZ = (ray.direction.z > 0) ? 1 : -1;

            float nextBoundX = aabb.min.x + (curGx + (stepX > 0 ? 1 : 0)) * cellSize;
            float nextBoundY = aabb.min.y + (curGy + (stepY > 0 ? 1 : 0)) * cellSize;
            float nextBoundZ = aabb.min.z + (curGz + (stepZ > 0 ? 1 : 0)) * cellSize;

            float tMaxX = (nextBoundX - ray.origin.x) * ray.inverseDirection.x;
            float tMaxY = (nextBoundY - ray.origin.y) * ray.inverseDirection.y;
            float tMaxZ = (nextBoundZ - ray.origin.z) * ray.inverseDirection.z;

            float tDeltaX = std::abs(cellSize * ray.inverseDirection.x);
            float tDeltaY = std::abs(cellSize * ray.inverseDirection.y);
            float tDeltaZ = std::abs(cellSize * ray.inverseDirection.z);

            while (tEntry <= tExit + cellSize * 0.1f)
            {
                uint64_t key = GetKey(curGx, curGy, curGz);
                auto it = gridHead.find(key);

                if (it != gridHead.end())
                {
                    int currIdx = it->second;
                    float minTInVoxel = FLT_MAX;
                    bool hitFound = false;

                    while (currIdx != -1)
                    {
                        float t = 0.0f;
                        if (ray.IntersectSphere(points[currIdx], pickRadius, t))
                        {
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

                    float tNextBoundary = std::min(std::min(tMaxX, tMaxY), tMaxZ);

                    if (hitFound && minTInVoxel <= tNextBoundary + 0.01f)
                    {
                        return result;
                    }
                }

                if (tMaxX < tMaxY) {
                    if (tMaxX < tMaxZ) {
                        curGx += stepX;
                        tEntry = tMaxX;
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

    class IGeometricProcessingOperatorBase {};

	template<typename SpatialPartitioningType>
	class IGeometricProcessingOperator : public IGeometricProcessingOperatorBase
    {
    public:
		IGeometricProcessingOperator(bool needToRebuildSpatialPartitioning) : needToRebuildSpatialPartitioning(needToRebuildSpatialPartitioning) {}
		virtual ~IGeometricProcessingOperator()
        {
            if (needToDeleteSpatialPartitioning)
            {
                SAFE_DELETE(spatialPartitioning);
            }
        }

        virtual void Process(PointCloud& pointCloud, SpatialPartitioningType* spatialPartitioning) = 0;
		virtual void Visualize() = 0;

        inline bool IsNeedToRebuildSpatialPartitioning() const { return needToRebuildSpatialPartitioning; }

    protected:
		bool needToDeleteSpatialPartitioning = false;
		bool needToRebuildSpatialPartitioning = false;
        SpatialPartitioningType* spatialPartitioning = nullptr;
        std::vector<int> pointTags;
		PointCloud* cachedPointCloud = nullptr;
	};

    class OperatorFilterETC : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorFilterETC(bool needToRebuildSpatialPartitioning = true)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
		}

        virtual void Process(PointCloud& pointCloud, SparseGrid* sparseGrid) override
        {
            TS(FilterETC);
            if (pointCloud.numberOfElements == 0) return;
            size_t writeIdx = 0;
            for (size_t readIdx = 0; readIdx < pointCloud.numberOfElements; ++readIdx)
            {
                int classID = pointCloud.pointClassIDs[readIdx];
                if (DL_ETC == classID)
                {
                    continue;
                }
                if (writeIdx != readIdx)
                {
                    pointCloud.positions[writeIdx] = pointCloud.positions[readIdx];
                    pointCloud.normals[writeIdx] = pointCloud.normals[readIdx];
                    pointCloud.colors[writeIdx] = pointCloud.colors[readIdx];
                    pointCloud.pointClassIDs[writeIdx] = pointCloud.pointClassIDs[readIdx];
                }
                writeIdx++;
            }
            pointCloud.numberOfElements = writeIdx;
            pointCloud.positions.resize(writeIdx);
            pointCloud.normals.resize(writeIdx);
			pointCloud.colors.resize(writeIdx);
            pointCloud.pointClassIDs.resize(writeIdx);
            //alog("Filtered ETC points. Remaining points: %s\n", FormatWithCommas(writeIdx).c_str());
			TE(FilterETC);
        }

        virtual void Visualize() override
        {
            // 없음
        }
    };

    class OperatorClustering : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorClustering(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        struct AtomicDisjointSet
        {
            std::unique_ptr<std::atomic<int>[]> parent;
            size_t size = 0;

            void Initialize(size_t n)
            {
                size = n;
                parent = std::make_unique<std::atomic<int>[]>(n);

                std::vector<int> indices(n);
                std::iota(indices.begin(), indices.end(), 0);

                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i) {
                    parent[i].store(i, std::memory_order_relaxed);
                    });
            }

            // Lock-Free Find (Halving)
            int Find(int i)
            {
                int p = parent[i].load(std::memory_order_relaxed);
                while (p != i)
                {
                    int pp = parent[p].load(std::memory_order_relaxed);
                    parent[i].store(pp, std::memory_order_relaxed); // Path splitting
                    i = pp;
                    p = parent[i].load(std::memory_order_relaxed);
                }
                return i;
            }

            void Union(int i, int j)
            {
                int rootA = Find(i);
                int rootB = Find(j);

                while (rootA != rootB)
                {
                    if (rootA > rootB) std::swap(rootA, rootB);

                    int expected = rootB;
                    if (parent[rootB].compare_exchange_weak(expected, rootA))
                    {
                        return;
                    }

                    rootA = Find(rootA);
                    rootB = Find(expected);
                }
            }
        };

        virtual void Process(PointCloud& pointCloud, SparseGrid* sparseGrid) override
        {
            TS(Clustering_Parallel);

            if (pointCloud.numberOfElements == 0) return;
            if (nullptr == sparseGrid)
            {
                sparseGrid = new SparseGrid();
                sparseGrid->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numberOfPoints = pointCloud.numberOfElements;
            pointClusterIDs.assign(numberOfPoints, -1);

            AtomicDisjointSet dsu;
            dsu.Initialize(numberOfPoints);

            float searchRadius = sparseGrid->cellSize * searchRadiusMultiplier;
            float searchRadiusSq = searchRadius * searchRadius;

            float strictAngleThreshold = 0.9f;

            float planeDistThreshold = sparseGrid->cellSize * 0.2f;

            struct CellData { uint64_t key; int headIdx; };
            std::vector<CellData> flatCells;
            flatCells.reserve(sparseGrid->gridHead.size());

            for (const auto& pair : sparseGrid->gridHead)
            {
                flatCells.push_back({ pair.first, pair.second });
            }

            const uint64_t mask = 0x1FFFFF;

            std::for_each(std::execution::par, flatCells.begin(), flatCells.end(), [&](const CellData& cell)
                {
                    uint64_t key = cell.key;
                    int headIdx = cell.headIdx;

                    int gz = (int)(key & mask);
                    int gy = (int)((key >> 21) & mask);
                    int gx = (int)(key >> 42);

                    for (int i = headIdx; i != -1; i = sparseGrid->nextPoint[i])
                    {
                        const glm::vec3& pA = pointCloud.positions[i];
                        const glm::vec3& nA = pointCloud.normals[i];

                        for (int j = sparseGrid->nextPoint[i]; j != -1; j = sparseGrid->nextPoint[j])
                        {
                            const glm::vec3& pB = pointCloud.positions[j];

                            if (glm::distance2(pA, pB) > searchRadiusSq) continue;

                            const glm::vec3& nB = pointCloud.normals[j];

                            if (glm::dot(nA, nB) < strictAngleThreshold) continue;

                            float planeDist = std::abs(glm::dot(nA, pB - pA));
                            if (planeDist > planeDistThreshold) continue;

                            if (useMarksForClustering)
                            {
                                if (pointCloud.marks[i] != pointCloud.marks[j]) continue;
                            }

                            dsu.Union(i, j);
                        }

                        for (int dz = -1; dz <= 1; ++dz)
                        {
                            for (int dy = -1; dy <= 1; ++dy)
                            {
                                for (int dx = -1; dx <= 1; ++dx)
                                {
                                    if (dx == 0 && dy == 0 && dz == 0) continue;

                                    uint64_t neighborKey = sparseGrid->GetKey(gx + dx, gy + dy, gz + dz);
                                    if (neighborKey < key) continue; // 중복 방지

                                    auto it = sparseGrid->gridHead.find(neighborKey);
                                    if (it == sparseGrid->gridHead.end()) continue;

                                    int neighborHead = it->second;
                                    for (int j = neighborHead; j != -1; j = sparseGrid->nextPoint[j])
                                    {
                                        const glm::vec3& pB = pointCloud.positions[j];

                                        if (glm::distance2(pA, pB) > searchRadiusSq) continue;

                                        const glm::vec3& nB = pointCloud.normals[j];

                                        if (glm::dot(nA, nB) < strictAngleThreshold) continue;

                                        float planeDist = std::abs(glm::dot(nA, pB - pA));
                                        if (planeDist > planeDistThreshold) continue;

                                        if (useMarksForClustering)
                                        {
                                            if (pointCloud.marks[i] != pointCloud.marks[j]) continue;
                                        }

                                        dsu.Union(i, j);
                                    }
                                }
                            }
                        }
                    }
                });

            std::map<int, int> rootToClusterID;
            int currentClusterCount = 0;

            for (size_t i = 0; i < numberOfPoints; ++i)
            {
                int root = dsu.Find((int)i);
                if (rootToClusterID.find(root) == rootToClusterID.end())
                {
                    rootToClusterID[root] = currentClusterCount++;
                }
                pointClusterIDs[i] = rootToClusterID[root];
            }

            //alog("Parallel Clustering Done. Found %d clusters.\n", currentClusterCount);
            TE(Clustering_Parallel);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud || pointClusterIDs.empty()) return;

            auto contrastingColors = Color::GetContrastingColors(64);

            size_t count = cachedPointCloud->numberOfElements;
            for (size_t i = 0; i < count; ++i)
            {
                int clusterId = pointClusterIDs[i];
                if (clusterId < 0) continue;

                const glm::vec3& p = cachedPointCloud->positions[i];

                VD::AddSphere(
                    "ClusteredPoints",
                    p,
                    Configuration::pointVisualizationRadius,
                    contrastingColors[clusterId % contrastingColors.size()]
                );
            }
        }

		inline bool IsUseMarksForClustering() const { return useMarksForClustering; }
		inline void SetUseMarksForClustering(bool useMarks) { useMarksForClustering = useMarks; }
    protected:
        std::vector<int> pointClusterIDs;
        float searchRadiusMultiplier = 1.5f;
		bool useMarksForClustering = false;
    };

    class OperatorClusteringComplex : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorClusteringComplex(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
		}

        struct ClusteringParams
        {
            float searchRadiusMult = 1.5f;        // 복셀 크기 대비 검색 반경
            float angleThreshold = 0.9f;          // 법선 내적 임계값 (0.9 = 약 25도)
            float planeOffsetThreshold = 0.2f;    // 평면 이격 거리 (복셀 크기 대비 비율)
            float colorThreshold = 0.15f;         // 색상 차이 임계값 (0.0 ~ 1.0)
            float curvatureDiffThreshold = 0.05f; // 곡률 차이 임계값
            bool useDeepLearningClasses = true;   // [NEW] DL 클래스 ID 활용 여부
        };

        ClusteringParams params;

        std::vector<int> pointClusterIds;
        std::vector<float> pointCurvatures;

        // Union-Find (Atomic)
        struct AtomicDisjointSet
        {
            std::unique_ptr<std::atomic<int>[]> parent;
            void Initialize(size_t n)
            {
                parent = std::make_unique<std::atomic<int>[]>(n);
                std::vector<int> indices(n);
                std::iota(indices.begin(), indices.end(), 0);
                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i) {
                    parent[i].store(i, std::memory_order_relaxed);
                    });
            }
            int Find(int i)
            {
                int p = parent[i].load(std::memory_order_relaxed);
                while (p != i) {
                    int pp = parent[p].load(std::memory_order_relaxed);
                    parent[i].store(pp, std::memory_order_relaxed);
                    i = pp;
                    p = parent[i].load(std::memory_order_relaxed);
                }
                return i;
            }
            void Union(int i, int j)
            {
                int rootA = Find(i);
                int rootB = Find(j);
                while (rootA != rootB) {
                    if (rootA > rootB) std::swap(rootA, rootB);
                    int expected = rootB;
                    if (parent[rootB].compare_exchange_weak(expected, rootA)) return;
                    rootA = Find(rootA);
                    rootB = Find(expected);
                }
            }
        };

        virtual void Process(PointCloud& pointCloud, SparseGrid* sparseGrid) override
        {
            TS(ComplexClustering);

            if (pointCloud.numberOfElements == 0) return;
            if (nullptr == sparseGrid)
            {
                sparseGrid = new SparseGrid();
                sparseGrid->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;
            pointClusterIds.assign(numPoints, -1);
            pointCurvatures.resize(numPoints);

            // DL Class ID 데이터 유효성 확인
            bool hasValidClassIDs = params.useDeepLearningClasses &&
                !pointCloud.pointClassIDs.empty() &&
                (pointCloud.pointClassIDs.size() == numPoints);

            // ----------------------------------------------------------------
            // Phase 1: 곡률(Curvature) 및 로컬 특징 사전 계산 (병렬)
            // ----------------------------------------------------------------
            TS(PrecalcFeatures);
            {
                std::vector<int> indices(numPoints);
                std::iota(indices.begin(), indices.end(), 0);
                float curvSearchRadSq = std::pow(sparseGrid->cellSize * 2.0f, 2.0f);

                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                    {
                        glm::vec3 center = pointCloud.positions[i];
                        int neighbors = 0;

                        int gx = (int)std::floor((center.x - sparseGrid->aabb.min.x) / sparseGrid->cellSize);
                        int gy = (int)std::floor((center.y - sparseGrid->aabb.min.y) / sparseGrid->cellSize);
                        int gz = (int)std::floor((center.z - sparseGrid->aabb.min.z) / sparseGrid->cellSize);

                        float sumDistSq = 0.0f;

                        for (int dz = -1; dz <= 1; ++dz) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dx = -1; dx <= 1; ++dx) {
                                    uint64_t key = sparseGrid->GetKey(gx + dx, gy + dy, gz + dz);
                                    auto it = sparseGrid->gridHead.find(key);
                                    if (it == sparseGrid->gridHead.end()) continue;

                                    for (int idx = it->second; idx != -1; idx = sparseGrid->nextPoint[idx]) {
                                        if (i == idx) continue;
                                        if (glm::distance2(center, pointCloud.positions[idx]) <= curvSearchRadSq) {
                                            float dot = glm::dot(pointCloud.normals[i], pointCloud.normals[idx]);
                                            sumDistSq += (1.0f - std::abs(dot));
                                            neighbors++;
                                        }
                                    }
                                }
                            }
                        }
                        if (neighbors > 0) pointCurvatures[i] = std::min(1.0f, sumDistSq / (float)neighbors);
                        else pointCurvatures[i] = 0.0f;
                    });
            }
            TE(PrecalcFeatures);

            // ----------------------------------------------------------------
            // Phase 2: 복합 기준 클러스터링 (병렬 Union-Find)
            // ----------------------------------------------------------------
            TS(ClusteringExecution);

            AtomicDisjointSet dsu;
            dsu.Initialize(numPoints);

            float searchRadiusSq = std::pow(sparseGrid->cellSize * params.searchRadiusMult, 2.0f);
            float planeDistAbs = sparseGrid->cellSize * params.planeOffsetThreshold;

            struct CellData { uint64_t key; int headIdx; };
            std::vector<CellData> flatCells;
            flatCells.reserve(sparseGrid->gridHead.size());
            for (const auto& pair : sparseGrid->gridHead) flatCells.push_back({ pair.first, pair.second });

            const uint64_t mask = 0x1FFFFF;

            std::for_each(std::execution::par, flatCells.begin(), flatCells.end(), [&](const CellData& cell)
                {
                    uint64_t key = cell.key;
                    int gx = (int)(key >> 42);
                    int gy = (int)((key >> 21) & mask);
                    int gz = (int)(key & mask);

                    for (int i = cell.headIdx; i != -1; i = sparseGrid->nextPoint[i])
                    {
                        const glm::vec3& pA = pointCloud.positions[i];
                        const glm::vec3& nA = pointCloud.normals[i];
                        const glm::vec3& cA = pointCloud.colors[i];
                        float curvA = pointCurvatures[i];
                        int classA = hasValidClassIDs ? pointCloud.pointClassIDs[i] : -1;

                        auto CheckAndMerge = [&](int j)
                            {
                                // 0. Class ID Check (Hard Constraint) [NEW]
                                // 서로 다른 클래스(예: 치아 vs 잇몸)라면 기하학적으로 가까워도 무조건 분리
                                if (hasValidClassIDs)
                                {
                                    if (classA != pointCloud.pointClassIDs[j]) return;
                                }

                                const glm::vec3& pB = pointCloud.positions[j];

                                // 1. Distance Check
                                if (glm::distance2(pA, pB) > searchRadiusSq) return;

                                const glm::vec3& nB = pointCloud.normals[j];

                                // 2. Normal Direction
                                if (glm::dot(nA, nB) < params.angleThreshold) return;

                                // 3. Plane Offset
                                if (std::abs(glm::dot(nA, pB - pA)) > planeDistAbs) return;

                                // 4. Color Check
                                if (glm::distance(cA, pointCloud.colors[j]) > params.colorThreshold) return;

                                // 5. Curvature Consistency
                                if (std::abs(curvA - pointCurvatures[j]) > params.curvatureDiffThreshold) return;

                                dsu.Union(i, j);
                            };

                        // (A) Intra-Cell
                        for (int j = sparseGrid->nextPoint[i]; j != -1; j = sparseGrid->nextPoint[j]) {
                            CheckAndMerge(j);
                        }

                        // (B) Inter-Cell
                        for (int dz = -1; dz <= 1; ++dz) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dx = -1; dx <= 1; ++dx) {
                                    if (dx == 0 && dy == 0 && dz == 0) continue;
                                    uint64_t nKey = sparseGrid->GetKey(gx + dx, gy + dy, gz + dz);
                                    if (nKey < key) continue;

                                    auto it = sparseGrid->gridHead.find(nKey);
                                    if (it == sparseGrid->gridHead.end()) continue;

                                    for (int j = it->second; j != -1; j = sparseGrid->nextPoint[j]) {
                                        CheckAndMerge(j);
                                    }
                                }
                            }
                        }
                    }
                });

            // ----------------------------------------------------------------
            // Phase 3: 라벨링
            // ----------------------------------------------------------------
            std::map<int, int> rootToId;
            int clusterCount = 0;
            for (size_t i = 0; i < numPoints; ++i)
            {
                int root = dsu.Find((int)i);
                if (rootToId.find(root) == rootToId.end()) rootToId[root] = clusterCount++;
                pointClusterIds[i] = rootToId[root];
            }

            alog("Complex Clustering Done. Found %d clusters.\n", clusterCount);
            TE(ClusteringExecution);
            TE(ComplexClustering);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud || pointClusterIds.empty()) return;
            auto colors = Color::GetContrastingColors(64);

            for (size_t i = 0; i < cachedPointCloud->numberOfElements; ++i)
            {
                int id = pointClusterIds[i];
                if (id < 0) continue;

                VD::AddSphere("ComplexClusters",
                    cachedPointCloud->positions[i],
                    Configuration::pointVisualizationRadius,
                    colors[id % 64]);
            }
        }
    };

    class OperatorCurvatureEstimation : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorCurvatureEstimation(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud, SparseGrid* sparseGrid) override
        {
            TS(Curvature_Parallel);

            if (pointCloud.numberOfElements == 0) return;

            if (nullptr == sparseGrid)
            {
                sparseGrid = new SparseGrid();
                sparseGrid->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numberOfPoints = pointCloud.numberOfElements;
            curvatures.resize(numberOfPoints);

            float searchRadius = sparseGrid->cellSize * searchRadiusScale;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numberOfPoints);
            std::iota(indices.begin(), indices.end(), 0);

            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = pointCloud.positions[i];

                    std::vector<int> neighbors;
                    neighbors.reserve(64);

                    glm::vec3 centroid(0.0f);

                    int gx = (int)std::floor((p.x - sparseGrid->aabb.min.x) / sparseGrid->cellSize);
                    int gy = (int)std::floor((p.y - sparseGrid->aabb.min.y) / sparseGrid->cellSize);
                    int gz = (int)std::floor((p.z - sparseGrid->aabb.min.z) / sparseGrid->cellSize);

                    for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz)
                    {
                        for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy)
                        {
                            for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx)
                            {
                                uint64_t key = sparseGrid->GetKey(gx + dx, gy + dy, gz + dz);
                                auto it = sparseGrid->gridHead.find(key);
                                if (it == sparseGrid->gridHead.end()) continue;

                                int currIdx = it->second;
                                while (currIdx != -1)
                                {
                                    if (currIdx != i)
                                    {
                                        if (glm::distance2(p, pointCloud.positions[currIdx]) <= searchRadiusSq)
                                        {
                                            neighbors.push_back(currIdx);
                                            centroid += pointCloud.positions[currIdx];
                                        }
                                    }
                                    currIdx = sparseGrid->nextPoint[currIdx];
                                }
                            }
                        }
                    }

                    size_t k = neighbors.size();
                    if (k < 4)
                    {
                        curvatures[i] = 0.0f;
                        return;
                    }

                    centroid /= (float)k;

                    // Covariance Matrix (3x3 Symmetric)
                    // Cov = Sum( (p - c) * (p - c)^T )
                    float xx = 0, xy = 0, xz = 0;
                    float yy = 0, yz = 0, zz = 0;

                    for (int idx : neighbors)
                    {
                        glm::vec3 r = pointCloud.positions[idx] - centroid;
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

                    glm::vec3 evals = ComputeEigenValuesSymmetric(cov);

                    // 정렬 (lambda0 <= lambda1 <= lambda2)
                    float l0 = evals.x, l1 = evals.y, l2 = evals.z;
                    if (l0 > l1) std::swap(l0, l1);
                    if (l1 > l2) std::swap(l1, l2);
                    if (l0 > l1) std::swap(l0, l1);

                    // Surface Variation = lambda0 / (lambda0 + lambda1 + lambda2)
                    // 평면일수록 l0(법선 방향 분산)가 0에 가깝고, 엣지나 구형일수록 커짐
                    float sum = l0 + l1 + l2;
                    if (sum > 1e-9f)
                    {
                        curvatures[i] = l0 / sum;
                    }
                    else
                    {
                        curvatures[i] = 0.0f;
                    }

                    if(curvatureThreshold < curvatures[i])
                    {
						pointCloud.marks[i] = 1; // High Curvature Mark
                    }
                });

            TE(Curvature_Parallel);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud || curvatures.empty()) return;

            size_t count = cachedPointCloud->numberOfElements;

			auto [curvatureMin, curvatureMax] = std::minmax_element(curvatures.begin(), curvatures.end());

            for (size_t i = 0; i < count; ++i)
            {
                float val = curvatures[i];

                // 0.01 이하는 평면으로 보고 렌더링 생략 (성능 및 시인성 확보)
                // 필요하면 주석 해제하여 전체 렌더링
                // if (val < 0.01f) continue; 

                // Color Map: Blue(Low) -> Green -> Red(High)
                //float t = glm::clamp(val * visualScale, 0.0f, 1.0f);
				//float t = (val - *curvatureMin) / (*curvatureMax - *curvatureMin);
                //glm::vec3 color = glm::mix(glm::vec3(0, 0, 1), glm::vec3(1, 0, 0), t);

				glm::vec4 color = glm::vec4(cachedPointCloud->colors[i], 1.0f);
                if(cachedPointCloud->marks[i] == 1)
                {
                    color = Color::red();
				}
                else
                {
                    color = Color::blue();
                }

                VD::AddSphere(
                    "CurvatureEstimation",
                    cachedPointCloud->positions[i],
                    Configuration::pointVisualizationRadius, // 점 크기
                    color
                );
            }
        }

        inline float GetCurvatureThreshold() const { return curvatureThreshold; }
        inline void SetCurvatureThreshold(float threshold) { curvatureThreshold = threshold; }
        
        inline int GetNeighborSearchOffset() const { return neighborSearchOffset; }
		inline void SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; }

		inline float GetSearchRadiusScale() const { return searchRadiusScale; }
		inline void SetSearchRadiusScale(float scale) { searchRadiusScale = scale; }

		inline float GetVisualizationScale() const { return visualScale; }
        inline void SetVisualizationScale(float scale) { visualScale = scale; }
    private:
        float curvatureThreshold = 0.1f;
        int neighborSearchOffset = 1;
        float searchRadiusScale = 2.0f;
        float visualScale = 150.0f;

        std::vector<float> curvatures; // 0.0 ~ 1.0

        // 3x3 대칭 행렬 고유값 계산 (Cardan's method)
        inline glm::vec3 ComputeEigenValuesSymmetric(const glm::mat3& M)
        {
            double m = (M[0][0] + M[1][1] + M[2][2]) / 3.0;
            double p = (glm::pow(M[0][0] - m, 2.0) + glm::pow(M[1][1] - m, 2.0) + glm::pow(M[2][2] - m, 2.0) +
                2.0 * (glm::pow(M[0][1], 2.0) + glm::pow(M[0][2], 2.0) + glm::pow(M[1][2], 2.0))) / 6.0;

            double q = glm::determinant(M - glm::mat3(m)) / 2.0;
            double phi = 0.0;

            if (p > 1e-9)
            {
                phi = glm::atan(glm::sqrt(std::max(0.0, p * p * p - q * q)), q) / 3.0;
            }

            if (phi < 0) phi += 3.14159265358979323846 / 3.0;

            double eig1 = m + 2.0 * std::sqrt(p) * std::cos(phi);
            double eig2 = m + 2.0 * std::sqrt(p) * std::cos(phi + 2.0 * 3.14159265358979323846 / 3.0);
            double eig3 = 3.0 * m - eig1 - eig2;

            return glm::vec3((float)eig1, (float)eig2, (float)eig3);
        }
    };

    class OperatorCurvatureDivergence : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorCurvatureDivergence(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
		}

        std::vector<float> curvatures;
        std::vector<glm::vec3> gradients;
        std::vector<float> divergences;

        float searchRadiusMultiplier = 2.0f;
        float visualizationScale = 500.0f;

        virtual void Process(PointCloud& pointCloud, SparseGrid* sparseGrid) override
        {
            TS(CurvatureDivergence_Total);

            if (pointCloud.numberOfElements == 0) return;

            if (nullptr == sparseGrid)
            {
                sparseGrid = new SparseGrid();
                sparseGrid->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;

            curvatures.assign(numPoints, 0.0f);
            gradients.assign(numPoints, glm::vec3(0.0f));
            divergences.assign(numPoints, 0.0f);

            float searchRadius = sparseGrid->cellSize * searchRadiusMultiplier;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            TS(Pass1_Curvature);
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = pointCloud.positions[i];
                    std::vector<int> neighbors;
                    neighbors.reserve(32);
                    glm::vec3 centroid(0.0f);

                    ProcessNeighbors(i, p, *sparseGrid, searchRadiusSq, [&](int neighborIdx) {
                        neighbors.push_back(neighborIdx);
                        centroid += pointCloud.positions[neighborIdx];
                        });

                    size_t k = neighbors.size();
                    if (k < 4) return;

                    centroid /= (float)k;

                    float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
                    for (int idx : neighbors)
                    {
                        glm::vec3 r = pointCloud.positions[idx] - centroid;
                        xx += r.x * r.x; xy += r.x * r.y; xz += r.x * r.z;
                        yy += r.y * r.y; yz += r.y * r.z; zz += r.z * r.z;
                    }

                    glm::mat3 cov;
                    cov[0][0] = xx; cov[0][1] = xy; cov[0][2] = xz;
                    cov[1][0] = xy; cov[1][1] = yy; cov[1][2] = yz;
                    cov[2][0] = xz; cov[2][1] = yz; cov[2][2] = zz;
                    cov /= (float)k;

                    glm::vec3 evals = ComputeEigenValuesSymmetric(cov);
                    float l0 = evals.x, l1 = evals.y, l2 = evals.z;
                    if (l0 > l1) std::swap(l0, l1);
                    if (l1 > l2) std::swap(l1, l2);
                    if (l0 > l1) std::swap(l0, l1);

                    float sum = l0 + l1 + l2;
                    if (sum > 1e-9f) curvatures[i] = l0 / sum;
                });
            TE(Pass1_Curvature);

            TS(Pass2_Gradient);
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = pointCloud.positions[i];
                    float c_i = curvatures[i];
                    glm::vec3 gradSum(0.0f);
                    float weightSum = 0.0f;

                    ProcessNeighbors(i, p, *sparseGrid, searchRadiusSq, [&](int j) {
                        glm::vec3 diff = pointCloud.positions[j] - p;
                        float dist = glm::length(diff);
                        if (dist > 1e-6f)
                        {
                            glm::vec3 dir = diff / dist;
                            float c_diff = curvatures[j] - c_i;
                            float weight = 1.0f / dist;

                            gradSum += dir * (c_diff * weight);
                            weightSum += weight;
                        }
                        });

                    if (weightSum > 1e-6f) gradients[i] = gradSum / weightSum;
                });
            TE(Pass2_Gradient);

            TS(Pass3_Divergence);
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = pointCloud.positions[i];
                    const glm::vec3& g_i = gradients[i];
                    float divSum = 0.0f;
                    float count = 0.0f;

                    ProcessNeighbors(i, p, *sparseGrid, searchRadiusSq, [&](int j) {
                        glm::vec3 diff = pointCloud.positions[j] - p;
                        float dist = glm::length(diff);
                        if (dist > 1e-6f)
                        {
                            glm::vec3 dir = diff / dist;
                            glm::vec3 g_diff = gradients[j] - g_i;
                            divSum += glm::dot(g_diff, dir);
                            count += 1.0f;
                        }
                        });

                    if (count > 0.5f) divergences[i] = divSum / count;
                });
            TE(Pass3_Divergence);
            TE(CurvatureDivergence_Total);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud || divergences.empty()) return;

            size_t count = cachedPointCloud->numberOfElements;

            for (size_t i = 0; i < count; ++i)
            {
                float val = divergences[i];

                if (std::abs(val) < 0.0001f)
                {
                    VD::AddSphere(
                        "CurvatureDivergence_original",
                        cachedPointCloud->positions[i],
                        Configuration::pointVisualizationRadius,
                        glm::vec4(cachedPointCloud->colors[i], 1.0f)
                    );

                    continue;
                }

                float t = glm::clamp(val * visualizationScale + 0.5f, 0.0f, 1.0f);

                glm::vec3 color;
                if (t < 0.5f) {
                    float localT = t * 2.0f;
                    color = glm::mix(glm::vec3(0, 0, 1), glm::vec3(0.5f, 0.5f, 0.5f), localT);

                    VD::AddSphere(
                        "CurvatureDivergence_sink",
                        cachedPointCloud->positions[i],
                        Configuration::pointVisualizationRadius,
                        glm::vec4(color, 1.0f)
                    );
                }
                else {
                    float localT = (t - 0.5f) * 2.0f;
                    color = glm::mix(glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(1, 0, 0), localT);

                    VD::AddSphere(
                        "CurvatureDivergence_source",
                        cachedPointCloud->positions[i],
                        Configuration::pointVisualizationRadius,
                        glm::vec4(color, 1.0f)
                    );
                }
            }
        }

    private:
        
        template<typename Func>
        inline void ProcessNeighbors(int idx, const glm::vec3& p, const SparseGrid& sg, float rSq, Func func)
        {
            int gx = (int)std::floor((p.x - sg.aabb.min.x) / sg.cellSize);
            int gy = (int)std::floor((p.y - sg.aabb.min.y) / sg.cellSize);
            int gz = (int)std::floor((p.z - sg.aabb.min.z) / sg.cellSize);

            for (int dz = -1; dz <= 1; ++dz) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        uint64_t key = sg.GetKey(gx + dx, gy + dy, gz + dz);
                        auto it = sg.gridHead.find(key);
                        if (it == sg.gridHead.end()) continue;

                        int curr = it->second;
                        while (curr != -1) {
                            if (curr != idx) {
                                if (glm::distance2(p, cachedPointCloud->positions[curr]) <= rSq) {
                                    func(curr);
                                }
                            }
                            curr = sg.nextPoint[curr];
                        }
                    }
                }
            }
        }

        inline glm::vec3 ComputeEigenValuesSymmetric(const glm::mat3& M)
        {
            double m = (M[0][0] + M[1][1] + M[2][2]) / 3.0;
            double p = (glm::pow(M[0][0] - m, 2.0) + glm::pow(M[1][1] - m, 2.0) + glm::pow(M[2][2] - m, 2.0) +
                2.0 * (glm::pow(M[0][1], 2.0) + glm::pow(M[0][2], 2.0) + glm::pow(M[1][2], 2.0))) / 6.0;
            double q = glm::determinant(M - glm::mat3(m)) / 2.0;
            double phi = 0.0;
            if (p > 1e-9) phi = glm::atan(glm::sqrt(std::max(0.0, p * p * p - q * q)), q) / 3.0;
            if (phi < 0) phi += 3.14159265358979323846 / 3.0;
            double eig1 = m + 2.0 * std::sqrt(p) * std::cos(phi);
            double eig2 = m + 2.0 * std::sqrt(p) * std::cos(phi + 2.0 * 3.14159265358979323846 / 3.0);
            double eig3 = 3.0 * m - eig1 - eig2;
            return glm::vec3((float)eig1, (float)eig2, (float)eig3);
        }
    };

    class Pipeline
    {
    public:
        Pipeline() = default;
        ~Pipeline()
        {
            Clear();
		}

        void BuildSparseGrid(PointCloud& pointCloud)
        {
            SAFE_DELETE(sparseGrid);
            
            sparseGrid = new SparseGrid();
            sparseGrid->Build(pointCloud, Configuration::voxelSize);
		}

        template<typename OperatorType>
        std::shared_ptr<OperatorType> AddOperator(const std::string& tag, bool needToRebuildSpatialPartitioning)
        {
            auto op = std::make_shared<OperatorType>(needToRebuildSpatialPartitioning);
            operators.emplace_back(std::make_tuple(tag, op));
            return op;
        }

        void Execute(PointCloud& pointCloud)
        {
			TS(GeometricProcessingPipeline);

            for (auto& [tag, op] : operators)
            {
                {
                    auto time = std::chrono::high_resolution_clock::now();

                    op->Process(pointCloud, sparseGrid);

                    std::cout << Miliseconds(time, tag.c_str()) << std::endl;
                }

                {
                    auto time = std::chrono::high_resolution_clock::now();

                    if (op->IsNeedToRebuildSpatialPartitioning())
                    {
                        BuildSparseGrid(pointCloud);
                    }

                    std::cout << Miliseconds(time, "Rebuilding Spatial Grid") << std::endl;
                }
            }

            TE(GeometricProcessingPipeline);
        }

        void VisualizeAll()
        {
            for (auto& [tag, op] : operators)
            {
                op->Visualize();
            }
		}

        void Clear()
        {
            operators.clear();

            SAFE_DELETE(sparseGrid);
		}

        inline SparseGrid* GetSparseGrid() const { return sparseGrid; }

    protected:
        std::vector<std::tuple<std::string, std::shared_ptr<IGeometricProcessingOperator<SparseGrid>>>> operators;
        SparseGrid* sparseGrid = nullptr;
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

namespace GPP = GeometricProcessingPipeline;

int main(int argc, char** argv)
{
    std::cout << "AppFeather - Final Optimized" << std::endl;
    Feather.Initialize(1920, 1080);
    Feather.SetConsoleWindowIndex(3);
    Feather.SetMainWindowIndex(2);
    auto w = Feather.GetFeatherWindow();

    GeometricProcessingPipeline::Pipeline pipeline;
    GeometricProcessingPipeline::PointCloud pc;

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
            //else if (GLFW_KEY_F1 == event.keyCode && event.action == 0) VD::ToggleVisibility("SparseGridPoints");
            //else if (GLFW_KEY_F2 == event.keyCode && event.action == 0) VD::ToggleVisibility("SparseGridCells");
            else if (GLFW_KEY_F1 == event.keyCode && event.action == 0) VD::ToggleVisibility("CurvatureDivergence_original");
            else if (GLFW_KEY_F2 == event.keyCode && event.action == 0) VD::ToggleVisibility("CurvatureDivergence_sink");
            else if (GLFW_KEY_F3 == event.keyCode && event.action == 0) VD::ToggleVisibility("CurvatureDivergence_source");
            else if (GLFW_KEY_F9 == event.keyCode && event.action == 0)
            {
                std::ifstream in("camera_state.txt");
                if (in.is_open())
                {
                    glm::vec3 eye, target, up;
                    in >> eye.x >> eye.y >> eye.z;
                    in >> target.x >> target.y >> target.z;
                    in >> up.x >> up.y >> up.z;

                    auto camEnt = Feather.GetEntityByName("Camera");
                    if (camEnt != entt::null)
                    {
                        auto cam = Feather.GetComponent<Camera>(camEnt);
                        auto manipulator = Feather.GetComponent<CameraManipulatorTrackball>(camEnt);

                        if (cam)
                        {
                            cam->SetEye(eye);
                            cam->SetTarget(target);
                            cam->SetUp(up);
                            cam->SetDirty(true);

                            if (manipulator)
                            {
                                manipulator->SyncRadius();
                            }

                            std::cout << "[System] Camera state RESTORED." << std::endl;
                        }
                    }
                }
                else
                {
                    std::cout << "[System] No saved camera state file found." << std::endl;
                }
            }
            else if (GLFW_KEY_F12 == event.keyCode && event.action == 0)
            {
                auto camEnt = Feather.GetEntityByName("Camera");
                if (camEnt != entt::null)
                {
                    auto cam = Feather.GetComponent<Camera>(camEnt);
                    if (cam)
                    {
                        std::ofstream out("camera_state.txt");
                        if (out.is_open())
                        {
                            glm::vec3 eye = cam->GetEye();
                            glm::vec3 target = cam->GetTarget();
                            glm::vec3 up = cam->GetUp();

                            // Eye, Target, Up 순서로 저장
                            out << eye.x << " " << eye.y << " " << eye.z << std::endl;
                            out << target.x << " " << target.y << " " << target.z << std::endl;
                            out << up.x << " " << up.y << " " << up.z << std::endl;

                            std::cout << "[System] Camera state SAVED to 'camera_state.txt'" << std::endl;
                            std::cout << "  Eye: " << eye.x << ", " << eye.y << ", " << eye.z << std::endl;
                        }
                        else
                        {
                            std::cout << "[Error] Failed to open file for saving." << std::endl;
                        }
                    }
                }
            }
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

            if (event.button == GLFW_MOUSE_BUTTON_LEFT && event.action == 1)
            {
                VD::Clear("PickedPoint");
                VD::Clear("PickedCell");

                auto window = Feather.GetFeatherWindow();
                GLFWwindow* nativeWin = window->GetGLFWwindow();

                int winW, winH, fbW, fbH;
                glfwGetWindowSize(nativeWin, &winW, &winH);
                glfwGetFramebufferSize(nativeWin, &fbW, &fbH);

                double mx, my;
                glfwGetCursorPos(nativeWin, &mx, &my);

                float pxRatio = (float)fbW / (float)winW;
                float pyRatio = (float)fbH / (float)winH;
                mx *= pxRatio;
                my *= pyRatio;

                float ndcX = (2.0f * (float)mx) / (float)fbW - 1.0f;
                float ndcY = 1.0f - (2.0f * (float)my) / (float)fbH;

                auto cameraComp = Feather.GetComponent<Camera>(cam);
                glm::mat4 view = cameraComp->GetViewMatrix();
                glm::mat4 proj = cameraComp->GetProjectionMatrix();
                glm::mat4 invVP = glm::inverse(proj * view);

                glm::vec4 screenPosNear(ndcX, ndcY, -1.0f, 1.0f);
                glm::vec4 screenPosFar(ndcX, ndcY, 1.0f, 1.0f);

                glm::vec4 worldPosNear = invVP * screenPosNear;
                glm::vec4 worldPosFar = invVP * screenPosFar;

                if (worldPosNear.w != 0.0f) worldPosNear /= worldPosNear.w;
                if (worldPosFar.w != 0.0f) worldPosFar /= worldPosFar.w;

                glm::vec3 rayOrigin = glm::vec3(worldPosNear);
                glm::vec3 rayDir = glm::normalize(glm::vec3(worldPosFar - worldPosNear));

                Ray ray{ rayOrigin, rayDir };

                auto result = pipeline.GetSparseGrid()->Pick(pc.positions, ray, GeometricProcessingPipeline::Configuration::pointVisualizationRadius);

                if (result.hasHit)
                {
                    glm::vec3 p = pc.positions[result.pointIndex];
                    VD::AddSphere("PickedPoint", p, GeometricProcessingPipeline::Configuration::pointVisualizationRadius * 1.1f, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

                    if (Feather.IsKeyPressed(GLFW_KEY_LEFT_CONTROL) || Feather.IsKeyPressed(GLFW_KEY_RIGHT_CONTROL))
                    {
                        manipulator->SetCenter(p);
                    }

                    glm::vec3 cellMin = pipeline.GetSparseGrid()->aabb.min + glm::vec3(
                        (float)result.gx * pipeline.GetSparseGrid()->cellSize,
                        (float)result.gy * pipeline.GetSparseGrid()->cellSize,
                        (float)result.gz * pipeline.GetSparseGrid()->cellSize
                    );
                    glm::vec3 cellMax = cellMin + glm::vec3(pipeline.GetSparseGrid()->cellSize);
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

    Feather.AddOnInitializeCallback([&]()
        {
            {
                TS(PLYLoading);
                PLYFormat ply;
                if (!ply.Deserialize("D:\\Debug\\PLY\\Compound_0.ply"))
                {
                    printf("Failed to load PLY.\n");
                    return;
                }
                pc.FromPLY(ply);
                TE(PLYLoading);
            }

            pipeline.BuildSparseGrid(pc);

            //pipeline.AddOperator<GPP::OperatorFilterETC>("OperatorFilterETC", true);
            //pipeline.AddOperator<GPP::OperatorCurvatureDivergence>("OperatorCurvatureDivergence", false);
            auto operatorCurvatureEstimation = pipeline.AddOperator<GPP::OperatorCurvatureEstimation>("OperatorCurvatureEstimation", false);
			operatorCurvatureEstimation->SetNeighborSearchOffset(3);
			operatorCurvatureEstimation->SetSearchRadiusScale(5.0f);
			operatorCurvatureEstimation->SetVisualizationScale(5.0f);

			auto operatorClustering = pipeline.AddOperator<GPP::OperatorClustering>("OperatorClustering", false);
            operatorClustering->SetUseMarksForClustering(true);

            pipeline.Execute(pc);

            //pipeline.VisualizeAll();
            //operatorCurvatureEstimation->Visualize();
			operatorClustering->Visualize();

            /*GPP::OperatorClustering operatorClustering;
            {
                TS(Clustering);
                operatorClustering.Process(pc, &sgrid);
                TE(Clustering);

                TS(Visualizing);
                operatorClustering.Visualize();
                TE(Visualizing);
            }*/

            /*GPP::OperatorCurvatureEstimation operatorCurvatureEstimation;
            {
                TS(Clustering);
                operatorCurvatureEstimation.Process(pc, &sgrid);
                TE(Clustering);

                TS(Visualizing);
                operatorCurvatureEstimation.Visualize();
                TE(Visualizing);
            }*/

            /*GPP::OperatorClusteringComplex operatorClusteringComplex;
            {
                TS(ClusteringComplex);
                operatorClusteringComplex.Process(pc, &sgrid);
                TE(ClusteringComplex);

                TS(VisualizingComplex);
                operatorClusteringComplex.Visualize();
                TE(VisualizingComplex);
            }*/
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

            for (size_t i = 0; i < rawCount; ++i)
            {
                float x = rawPts[i * 3], y = rawPts[i * 3 + 1], z = rawPts[i * 3 + 2];
                if (x >= GPP::Configuration::filterMin.x && x <= GPP::Configuration::filterMax.x &&
                    y >= GPP::Configuration::filterMin.y && y <= GPP::Configuration::filterMax.y &&
                    z >= GPP::Configuration::filterMin.z && z <= GPP::Configuration::filterMax.z)
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

            GPP::PointCloudCurvatureEstimator curvatureEstimator;
            curvatureEstimator.Process(points, 0.5f);
            curvatureEstimator.Visualize(points);

            GPP::PointCloudClusterer pcClusterer;
            
            pcClusterer.Process(points, normals, GPP::Configuration::voxelSize * 1.0f, 30.0f, 0.05f);
            //pcClusterer.Process_UnionAndFind(points, normals, Configuration::voxelSize * 1.3f, 15.0f);
            
            pcClusterer.Visualize(points, normals, colors);

            GPP::SparseDataBlock sdb;
            sdb.FromPointsData(points, normals, colors, pcClusterer.pointClusterIds, aabbMin);

            TS(MeshGeneration);
            GPP::MeshGenerator meshGen;
            meshGen.Generate(sdb);
            meshGen.DetectHoles();
            TE(MeshGeneration);

            meshGen.Visualize(true, true);
            //meshGen.ExportPLY("D:\\Debug\\PLY\\output.ply");

            sdb.Visualize();

            alog("Total DataBlocks : %s\n", FormatWithCommas(sdb.dataBlocks.size()).c_str());
            alog("DataBlock Size : %zd\n", sizeof(GPP::DataBlock));
            alog("Total Memory : %s bytes\n", FormatWithCommas(sdb.dataBlocks.size() * sizeof(GPP::DataBlock)).c_str());

            {
                size_t blockCount = sdb.dataBlocks.size();
                size_t blockSizeBytes = sizeof(GPP::DataBlock);
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
