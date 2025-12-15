#include <robin_hood.h>

#include <libFeather.h>

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
        std::vector<int> pointDeepLearningClassIDs;
        std::vector<int> pointClusterIDs;
        std::vector<int> marks;

        AABB aabb;

        void Clear()
        {
            numberOfElements = 0;
            positions.clear();
            normals.clear();
			colors.clear();
			pointDeepLearningClassIDs.clear();
			pointClusterIDs.clear();
            marks.clear();
            aabb = AABB();
		}

        [[nodiscard]] PointCloud Clone() const
        {
            PointCloud pc;
            pc.numberOfElements = numberOfElements;
            pc.positions = positions;
            pc.normals = normals;
            pc.colors = colors;
            pc.pointDeepLearningClassIDs = pointDeepLearningClassIDs;
            pc.pointClusterIDs = pointClusterIDs;
            pc.marks = marks;
            pc.aabb = aabb;
            return pc;
        }

        void FromPLY(const std::string& plyFileName)
        {
			PLYFormat ply;
			ply.Deserialize(plyFileName);
            FromPLY(ply);
        }

        void FromPLY(const PLYFormat& ply)
        {
            Clear();

            numberOfElements = ply.GetPoints().size() / 3;
            positions.resize(numberOfElements);
            normals.resize(numberOfElements);
            colors.resize(numberOfElements);
			pointDeepLearningClassIDs.resize(numberOfElements, -1);
			pointClusterIDs.resize(numberOfElements, -1);
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
                memcpy(pointDeepLearningClassIDs.data(), ply.GetDeepLearningClasses().data(), sizeof(int) * numberOfElements);
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
        robin_hood::unordered_flat_map<uint64_t, int> voxelPointListHead;
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

            voxelPointListHead.clear();
            voxelPointListHead.reserve(pc.numberOfElements);

            nextPoint.assign(pc.numberOfElements, -1);

            aabb.min -= glm::vec3(cellSize * 0.1f);
            aabb.max += glm::vec3(cellSize * 0.1f);

            for (int i = 0; i < (int)pc.numberOfElements; ++i)
            {
                int gx = (int)((pc.positions[i].x - aabb.min.x) / cellSize);
                int gy = (int)((pc.positions[i].y - aabb.min.y) / cellSize);
                int gz = (int)((pc.positions[i].z - aabb.min.z) / cellSize);

                uint64_t key = GetKey(gx, gy, gz);

                auto it = voxelPointListHead.find(key);

                if (it != voxelPointListHead.end())
                {
                    nextPoint[i] = it->second;
                    it->second = i;
                }
                else
                {
                    voxelPointListHead[key] = i;
                }
            }
        }

        int GetClosestPoint(const std::vector<glm::vec3>& points, const glm::vec3& queryPos, float& outDist)
        {
            outDist = FLT_MAX;
            if (points.empty()) return -1;

            // 쿼리 포인트의 그리드 인덱스 계산
            int startGx = (int)std::floor((queryPos.x - aabb.min.x) / cellSize);
            int startGy = (int)std::floor((queryPos.y - aabb.min.y) / cellSize);
            int startGz = (int)std::floor((queryPos.z - aabb.min.z) / cellSize);

            float minDistSq = FLT_MAX;
            int closestIdx = -1;

            int searchRadius = 0;
            const int maxSearchRadius = 100; // 안전 장치

            while (searchRadius < maxSearchRadius)
            {
                // 현재 반경(searchRadius)에 해당하는 껍질(Shell) 부분의 복셀들을 순회
                int minR = -searchRadius;
                int maxR = searchRadius;

                for (int dz = minR; dz <= maxR; ++dz)
                {
                    for (int dy = minR; dy <= maxR; ++dy)
                    {
                        for (int dx = minR; dx <= maxR; ++dx)
                        {
                            // 최적화: 내부 복셀은 이미 검사했으므로 건너뛰고 표면만 검사
                            if (searchRadius > 0 && abs(dx) != searchRadius && abs(dy) != searchRadius && abs(dz) != searchRadius)
                            {
                                continue;
                            }

                            uint64_t key = GetKey(startGx + dx, startGy + dy, startGz + dz);
                            auto it = voxelPointListHead.find(key);

                            if (it != voxelPointListHead.end())
                            {
                                int currIdx = it->second;
                                while (currIdx != -1)
                                {
                                    glm::vec3 diff = queryPos - points[currIdx];
                                    float sqDist = glm::dot(diff, diff);

                                    if (sqDist < minDistSq)
                                    {
                                        minDistSq = sqDist;
                                        closestIdx = currIdx;
                                    }
                                    currIdx = nextPoint[currIdx];
                                }
                            }
                        }
                    }
                }

                // [조기 종료 조건]
                // 현재까지 찾은 최소 거리가 현재 검색 박스의 경계(벽)까지의 거리보다 가깝다면,
                // 더 바깥쪽 복셀에는 이보다 가까운 점이 존재할 수 없으므로 탐색 종료.
                if (closestIdx != -1)
                {
                    // 현재 검색 범위의 월드 좌표 경계 계산
                    float minX = aabb.min.x + (startGx - searchRadius) * cellSize;
                    float maxX = aabb.min.x + (startGx + searchRadius + 1) * cellSize;
                    float minY = aabb.min.y + (startGy - searchRadius) * cellSize;
                    float maxY = aabb.min.y + (startGy + searchRadius + 1) * cellSize;
                    float minZ = aabb.min.z + (startGz - searchRadius) * cellSize;
                    float maxZ = aabb.min.z + (startGz + searchRadius + 1) * cellSize;

                    // 쿼리 포인트에서 현재 검색 박스 경계면까지의 최단 거리
                    float distToX = std::min(std::abs(queryPos.x - minX), std::abs(queryPos.x - maxX));
                    float distToY = std::min(std::abs(queryPos.y - minY), std::abs(queryPos.y - maxY));
                    float distToZ = std::min(std::abs(queryPos.z - minZ), std::abs(queryPos.z - maxZ));

                    float minDistToBoundary = std::min({ distToX, distToY, distToZ });

                    // 거리 제곱 비교 (sqrt 연산 최소화)
                    if (minDistSq < minDistToBoundary * minDistToBoundary)
                    {
                        break;
                    }
                }

                searchRadius++;
            }

            if (closestIdx != -1)
            {
                outDist = std::sqrt(minDistSq);
            }

            return closestIdx;
        }

        void GetKNearestNeighbors(const std::vector<glm::vec3>& points, const glm::vec3& queryPos, int k, std::vector<unsigned int>& outIndices, std::vector<float>& outDistances)
        {
            outIndices.clear();
            outDistances.clear();
            if (points.empty() || k <= 0) return;

            // (거리 제곱, 인덱스)를 저장하는 Max Heap 우선순위 큐
            // 큐의 top에는 항상 k개의 점 중 가장 먼 점이 위치합니다.
            std::priority_queue<std::pair<float, int>> pq;

            // 쿼리 포인트가 속한 그리드 좌표 계산
            int startGx = (int)std::floor((queryPos.x - aabb.min.x) / cellSize);
            int startGy = (int)std::floor((queryPos.y - aabb.min.y) / cellSize);
            int startGz = (int)std::floor((queryPos.z - aabb.min.z) / cellSize);

            // 검색 반경 (복셀 단위)
            int searchRadius = 0;
            const int maxSearchRadius = 100; // 무한 루프 방지를 위한 안전 장치

            while (searchRadius < maxSearchRadius)
            {
                // 현재 반경(searchRadius)에 해당하는 껍질(Shell) 부분의 복셀들을 순회
                int minR = -searchRadius;
                int maxR = searchRadius;

                for (int dz = minR; dz <= maxR; ++dz)
                {
                    for (int dy = minR; dy <= maxR; ++dy)
                    {
                        for (int dx = minR; dx <= maxR; ++dx)
                        {
                            // 최적화: 내부 복셀은 이미 이전 루프에서 검사했으므로 건너뛰고, 현재 반경의 '표면'만 검사
                            if (searchRadius > 0 && abs(dx) != searchRadius && abs(dy) != searchRadius && abs(dz) != searchRadius)
                            {
                                continue;
                            }

                            uint64_t key = GetKey(startGx + dx, startGy + dy, startGz + dz);
                            auto it = voxelPointListHead.find(key);

                            if (it != voxelPointListHead.end())
                            {
                                int currIdx = it->second;
                                while (currIdx != -1)
                                {
                                    glm::vec3 diff = queryPos - points[currIdx];
                                    float sqDist = glm::dot(diff, diff);

                                    if (pq.size() < (size_t)k)
                                    {
                                        pq.push({ sqDist, currIdx });
                                    }
                                    else if (sqDist < pq.top().first)
                                    {
                                        pq.pop();
                                        pq.push({ sqDist, currIdx });
                                    }
                                    currIdx = nextPoint[currIdx];
                                }
                            }
                        }
                    }
                }

                // 종료 조건 검사:
                // k개를 모두 찾았고, 현재 검색한 범위(박스)의 경계까지의 거리가
                // 찾은 점들 중 가장 먼 점(pq.top)보다 멀다면 더 이상 검색할 필요가 없음.
                if (pq.size() == (size_t)k)
                {
                    // 현재 검색 범위의 월드 좌표 경계 계산
                    float minX = aabb.min.x + (startGx - searchRadius) * cellSize;
                    float maxX = aabb.min.x + (startGx + searchRadius + 1) * cellSize;
                    float minY = aabb.min.y + (startGy - searchRadius) * cellSize;
                    float maxY = aabb.min.y + (startGy + searchRadius + 1) * cellSize;
                    float minZ = aabb.min.z + (startGz - searchRadius) * cellSize;
                    float maxZ = aabb.min.z + (startGz + searchRadius + 1) * cellSize;

                    // 쿼리 포인트에서 현재 검색 박스 경계면(벽)까지의 최단 거리 계산
                    float distToX = std::min(std::abs(queryPos.x - minX), std::abs(queryPos.x - maxX));
                    float distToY = std::min(std::abs(queryPos.y - minY), std::abs(queryPos.y - maxY));
                    float distToZ = std::min(std::abs(queryPos.z - minZ), std::abs(queryPos.z - maxZ));

                    float minDistToBoundary = std::min({ distToX, distToY, distToZ });

                    // 경계까지의 거리가 현재 확보한 k번째 이웃의 거리보다 크면, 바깥쪽 복셀에는 더 가까운 점이 있을 수 없음.
                    if (minDistToBoundary * minDistToBoundary > pq.top().first)
                    {
                        break;
                    }
                }

                searchRadius++;
            }

            // 결과 저장 (우선순위 큐는 Max Heap이므로 거꾸로 꺼내야 오름차순 정렬됨)
            size_t count = pq.size();
            outIndices.resize(count);
            outDistances.resize(count);

            for (int i = (int)count - 1; i >= 0; --i)
            {
                outIndices[i] = pq.top().second;
                outDistances[i] = std::sqrt(pq.top().first);
                pq.pop();
            }
        }

        SparseGridPickResult Pick(const std::vector<glm::vec3>& points, const Ray& ray, float pickRadius)
        {
            SparseGridPickResult result;
            if (points.empty() || voxelPointListHead.empty()) return result;

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
                auto it = voxelPointListHead.find(key);

                if (it != voxelPointListHead.end())
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

            for (const auto& pair : voxelPointListHead)
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
            blockSizePerAxis = voxelSize * Configuration::voxelsPerBlockAxis;

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

        virtual void Process(PointCloud& pointCloud) = 0;
        void Process(PointCloud& pointCloud, SpatialPartitioningType* spatialPartitioning)
        {
			this->spatialPartitioning = spatialPartitioning;
			Process(pointCloud);
        }

		virtual void Visualize() = 0;

		inline SpatialPartitioningType* GetSpatialPartitioning() const { return spatialPartitioning; }

        inline bool IsNeedToRebuildSpatialPartitioning() const { return needToRebuildSpatialPartitioning; }

    protected:
		bool needToDeleteSpatialPartitioning = false;
		bool needToRebuildSpatialPartitioning = false;
        SpatialPartitioningType* spatialPartitioning = nullptr;
        std::vector<int> pointTags;
		PointCloud* cachedPointCloud = nullptr;
	};

    class OperatorPointCloudLoader : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudLoader(bool needToRebuildSpatialPartitioning = false)
			: IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
		{
		}

		virtual void Process(PointCloud& pointCloud) override
		{
            if (plyFilename.empty()) return;

			pointCloud.FromPLY(plyFilename);

			TS(PointCloudLoader);
			if (pointCloud.numberOfElements == 0) return;
			if (nullptr == spatialPartitioning)
			{
				spatialPartitioning = new SparseGrid();
				spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
				needToDeleteSpatialPartitioning = true;
			}
			cachedPointCloud = &pointCloud;
			TE(PointCloudLoader);
		}

		virtual void Visualize() override
		{
            for (size_t i = 0; i < cachedPointCloud->numberOfElements; i++)
            {
				const auto& p = cachedPointCloud->positions[i];
				const auto& n = cachedPointCloud->normals[i];
				const auto& c = cachedPointCloud->colors[i];

                VD::AddSphere("PointCloudLoader",
                    p,
                    Configuration::pointVisualizationRadius,
                    glm::vec4(c, 1.0f)
				);
            }
		}

		inline const std::string& GetPLYFilename() const { return plyFilename; }
		inline void SetPLYFilename(const std::string& filename) { plyFilename = filename; }

    protected:
		std::string plyFilename;
	};

    class OperatorPointCloudVisualization : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorPointCloudVisualization(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }
        
        virtual void Process(PointCloud& pointCloud) override
        {
            TS(PointCloudVisualization);
            if (pointCloud.numberOfElements == 0) return;
            // 1. Build Spatial Partitioning
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }
            cachedPointCloud = &pointCloud;
            TE(PointCloudVisualization);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud) return;
            size_t count = cachedPointCloud->numberOfElements;
            for (size_t i = 0; i < count; ++i)
            {
                VD::AddSphere(
                    "PointCloudVisualization",
                    cachedPointCloud->positions[i],
                    Configuration::pointVisualizationRadius * 0.9f,
                    //glm::vec4(cachedPointCloud->colors[i], 1.0f)

                    Color::red()
                );
            }
        }
	};

    class OperatorPointCloudLaplacianSmoothing : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorPointCloudLaplacianSmoothing(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(LaplacianSmoothing);

            if (pointCloud.numberOfElements == 0) return;

            // 1. Build Spatial Partitioning (한 번 빌드하여 모든 반복에 사용)
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;

            float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // 위치 핑퐁(Ping-Pong)을 위한 버퍼
            std::vector<glm::vec3> nextPositions = pointCloud.positions;

            // 2. Iterative Smoothing
            for (int iter = 0; iter < iterations; ++iter)
            {
                // 병렬 처리: 각 점에 대해 이웃의 평균 위치 계산
                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                    {
                        // 마킹된 포인트(예: 경계점)는 이동하지 않음 (옵션)
                        if (preserveMarks && !pointCloud.marks.empty() && pointCloud.marks[i] != 0)
                        {
                            nextPositions[i] = pointCloud.positions[i];
                            return;
                        }

                        const glm::vec3& p = pointCloud.positions[i];
                        glm::vec3 centroid(0.0f);
                        int neighborCount = 0;

                        // SparseGrid를 이용한 이웃 검색
                        int gx = (int)std::floor((p.x - spatialPartitioning->aabb.min.x) / spatialPartitioning->cellSize);
                        int gy = (int)std::floor((p.y - spatialPartitioning->aabb.min.y) / spatialPartitioning->cellSize);
                        int gz = (int)std::floor((p.z - spatialPartitioning->aabb.min.z) / spatialPartitioning->cellSize);

                        for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz)
                        {
                            for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy)
                            {
                                for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx)
                                {
                                    uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
                                    auto it = spatialPartitioning->voxelPointListHead.find(key);

                                    if (it == spatialPartitioning->voxelPointListHead.end()) continue;

                                    int curr = it->second;
                                    while (curr != -1)
                                    {
                                        // 자기 자신 제외하고 거리 체크
                                        if (curr != i)
                                        {
                                            // 현재 iteration의 위치 기준이 아닌, 이전 iteration(pointCloud.positions) 기준으로 이웃 판단
                                            if (glm::distance2(p, pointCloud.positions[curr]) <= searchRadiusSq)
                                            {
                                                centroid += pointCloud.positions[curr];
                                                neighborCount++;
                                            }
                                        }
                                        curr = spatialPartitioning->nextPoint[curr];
                                    }
                                }
                            }
                        }

                        if (neighborCount > 0)
                        {
                            centroid /= (float)neighborCount;
                            glm::vec3 delta = centroid - p;

                            // P_new = P_old + lambda * (Avg_Neighbors - P_old)
                            nextPositions[i] = p + delta * smoothingFactor;
                        }
                        else
                        {
                            // 이웃이 없으면 제자리 유지
                            nextPositions[i] = p;
                        }
                    });

                // 위치 업데이트 (다음 iteration을 위해)
                pointCloud.positions = nextPositions;
            }

            TE(LaplacianSmoothing);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud) return;

            size_t count = cachedPointCloud->numberOfElements;
            for (size_t i = 0; i < count; ++i)
            {
                // 스무딩된 결과를 시각화
                VD::AddSphere(
                    "SmoothedPoints",
                    cachedPointCloud->positions[i],
                    Configuration::pointVisualizationRadius,
                    glm::vec4(cachedPointCloud->colors[i], 1.0f)
                );
            }
        }

        // Parameters Setters/Getters
        inline void SetIterations(int iter) { iterations = iter; }
        inline int GetIterations() const { return iterations; }

        inline void SetSmoothingFactor(float lambda) { smoothingFactor = glm::clamp(lambda, 0.0f, 1.0f); }
        inline float GetSmoothingFactor() const { return smoothingFactor; }

		inline float GetSearchRadiusMultiplier() const { return searchRadiusMultiplier; }
        inline void SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; }
        
		inline int GetNeighborSearchOffset() const { return neighborSearchOffset; }
		inline void SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; }
		
        inline bool IsPreserveMarks() const { return preserveMarks; }
        inline void SetPreserveMarks(bool preserve) { preserveMarks = preserve; }

    private:
        int iterations = 3;             // 반복 횟수
        float smoothingFactor = 0.5f;   // Lambda (0.0 ~ 1.0), 클수록 많이 이동
        float searchRadiusMultiplier = 1.5f;
        int neighborSearchOffset = 1;
        bool preserveMarks = true;      // marks가 0이 아닌 포인트 고정 여부
    };

    class OperatorKNNSmoothing : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorKNNSmoothing(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(KNNSmoothing);

            if (pointCloud.numberOfElements == 0) return;

            // 1. Build Spatial Partitioning if needed (used for KNN search)
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // 위치 업데이트를 위한 Ping-Pong 버퍼
            std::vector<glm::vec3> nextPositions = pointCloud.positions;

            // 2. Iterative Smoothing
            for (int iter = 0; iter < iterations; ++iter)
            {
                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                    {
                        // 마킹된 포인트(예: 특징점)는 이동하지 않음 (옵션)
                        //if (preserveMarks && !pointCloud.marks.empty() && pointCloud.marks[i] != 0)
                        //{
                        //    nextPositions[i] = pointCloud.positions[i];
                        //    return;
                        //}

                        if (false == (preserveMarks && !pointCloud.marks.empty() && pointCloud.marks[i] != 0))
                        {
                            return;
                        }

                        const glm::vec3& p = pointCloud.positions[i];

						pointCloud.colors[i] = glm::vec3(1.0f, 0.0f, 0.0f);

                        // Thread-local vectors for KNN result
                        // SparseGrid::GetKNearestNeighbors requires these containers
                        std::vector<unsigned int> neighborIndices;
                        std::vector<float> neighborDistances;
                        neighborIndices.reserve(kNeighbors);
                        neighborDistances.reserve(kNeighbors);

                        // Find K-Nearest Neighbors
                        // Note: This includes the point itself usually, depending on implementation.
                        // The provided GetKNearestNeighbors implementation in SparseGrid finds strictly closest points.
                        spatialPartitioning->GetKNearestNeighbors(
                            pointCloud.positions,
                            p,
                            kNeighbors,
                            neighborIndices,
                            neighborDistances
                        );

                        if (!neighborIndices.empty())
                        {
                            glm::vec3 centroid(0.0f);
                            float validCount = 0.0f;

                            for (unsigned int idx : neighborIndices)
                            {
                                // 자기 자신은 제외하고 평균을 구하고 싶다면 아래 조건 활성화
                                //if ((int)idx == i) continue; 

                                centroid += pointCloud.positions[idx];
                                validCount += 1.0f;
                            }

                            if (validCount > 0.0f)
                            {
                                centroid /= validCount;
                                glm::vec3 delta = centroid - p;

                                // Update position towards centroid
                                nextPositions[i] = p + delta * smoothingFactor;
                            }
                            else
                            {
                                nextPositions[i] = p;
                            }
                        }
                        else
                        {
                            nextPositions[i] = p;
                        }
                    });

                // Update positions for next iteration
                pointCloud.positions = nextPositions;
            }

            TE(KNNSmoothing);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud) return;

            size_t count = cachedPointCloud->numberOfElements;
            for (size_t i = 0; i < count; ++i)
            {
                VD::AddSphere(
                    "KNNSmoothedPoints",
                    cachedPointCloud->positions[i],
                    Configuration::pointVisualizationRadius,
                    glm::vec4(cachedPointCloud->colors[i], 1.0f)
                );
            }
        }

        // Parameters
        inline void SetK(int k) { kNeighbors = k; }
        inline int GetK() const { return kNeighbors; }

        inline void SetIterations(int iter) { iterations = iter; }
        inline int GetIterations() const { return iterations; }

        inline void SetSmoothingFactor(float factor) { smoothingFactor = glm::clamp(factor, 0.0f, 1.0f); }
        inline float GetSmoothingFactor() const { return smoothingFactor; }

        inline void SetPreserveMarks(bool preserve) { preserveMarks = preserve; }

    private:
        int kNeighbors = 8;
        int iterations = 3;
        float smoothingFactor = 0.5f;
        bool preserveMarks = true;
    };

    class OperatorSurfaceFitting : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorSurfaceFitting(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(SurfaceFitting);

            if (pointCloud.numberOfElements == 0) return;

            // 1. Build Spatial Partitioning if needed
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;

            for (int i = 0; i < iteration; i++)
            {
                // 결과를 담을 임시 버퍼 (Ping-Pong)
                std::vector<glm::vec3> newPositions = pointCloud.positions;
                std::vector<glm::vec3> newNormals = pointCloud.normals;

                std::vector<int> indices(numPoints);
                std::iota(indices.begin(), indices.end(), 0);

                // 2. Perform Surface Fitting (Parallel)
                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                    {
                        // 마킹된 포인트 보호 (옵션)
                        //if (preserveMarks && !pointCloud.marks.empty() && pointCloud.marks[i] != 0)
                        //{
                        //    return;
                        //}

                        const glm::vec3& p = pointCloud.positions[i];

                        // KNN Search
                        // SparseGrid::GetKNearestNeighbors requires these containers
                        std::vector<unsigned int> neighborIndices;
                        std::vector<float> neighborDistances;
                        neighborIndices.reserve(kNeighbors);
                        neighborDistances.reserve(kNeighbors);

                        spatialPartitioning->GetKNearestNeighbors(
                            pointCloud.positions,
                            p,
                            kNeighbors,
                            neighborIndices,
                            neighborDistances
                        );

                        size_t k = neighborIndices.size();
                        if (k < 4) return; // 점이 너무 적으면 피팅 불가

                        // --- Weighted PCA (Principal Component Analysis) ---

                        // 1) Compute Weighted Centroid
                        glm::vec3 centroid(0.0f);
                        float totalWeight = 0.0f;

                        // 가우시안 가중치 계산을 위한 파라미터 (가장 먼 이웃 거리 기준)
                        float maxDist = neighborDistances.back();
                        float h = std::max(maxDist * 0.5f, 1e-6f); // bandwidth
                        float hSq = h * h;

                        std::vector<float> weights(k);

                        for (size_t j = 0; j < k; ++j)
                        {
                            float distSq = neighborDistances[j] * neighborDistances[j];
                            float w = std::exp(-distSq / hSq); // Gaussian Kernel

                            weights[j] = w;
                            centroid += pointCloud.positions[neighborIndices[j]] * w;
                            totalWeight += w;
                        }

                        if (totalWeight < 1e-6f) return;
                        centroid /= totalWeight;

                        // 2) Compute Weighted Covariance Matrix
                        // Cov = Sum( w_i * (p_i - c) * (p_i - c)^T )
                        float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

                        for (size_t j = 0; j < k; ++j)
                        {
                            glm::vec3 r = pointCloud.positions[neighborIndices[j]] - centroid;
                            float w = weights[j];

                            xx += w * r.x * r.x;
                            xy += w * r.x * r.y;
                            xz += w * r.x * r.z;
                            yy += w * r.y * r.y;
                            yz += w * r.y * r.z;
                            zz += w * r.z * r.z;
                        }

                        glm::mat3 cov;
                        cov[0][0] = xx; cov[0][1] = xy; cov[0][2] = xz;
                        cov[1][0] = xy; cov[1][1] = yy; cov[1][2] = yz;
                        cov[2][0] = xz; cov[2][1] = yz; cov[2][2] = zz;

                        cov /= totalWeight;

                        // 3) Solve Eigen System to find Plane Normal
                        // 가장 작은 고유값에 해당하는 고유벡터가 평면의 법선(Normal)입니다.
                        glm::vec3 eigenVals;
                        glm::mat3 eigenVecs;
                        ComputeEigenDecomposition(cov, eigenVals, eigenVecs);

                        // 고유값은 오름차순 정렬되어 있다고 가정 (ComputeEigenDecomposition 내부 처리)
                        // eigenVecs[0] -> smallest eigenvalue's vector (Estimated Normal)
                        glm::vec3 planeNormal = eigenVecs[0];

                        // 법선 방향 일관성 유지 (기존 법선과 내적하여 뒤집힘 방지)
                        if (glm::dot(planeNormal, pointCloud.normals[i]) < 0.0f)
                        {
                            planeNormal = -planeNormal;
                        }

                        // 4) Project Point onto the Plane
                        // Plane defined by (Centroid, planeNormal)
                        // Projected P' = P - dot(P - Centroid, Normal) * Normal
                        glm::vec3 diff = p - centroid;
                        float distToPlane = glm::dot(diff, planeNormal);

                        // 이동 벡터 제한 (너무 급격한 변화 방지)
                        //float moveLimit = spatialPartitioning->cellSize * 0.5f;
                        //distToPlane = glm::clamp(distToPlane, -moveLimit, moveLimit);

                        newPositions[i] = p - planeNormal * distToPlane;

                        if (updateNormals)
                        {
                            newNormals[i] = planeNormal;
                        }

                        if (0.1f < glm::length(newPositions[i] - p))
                        {
                            pointCloud.colors[i] = Color::red();
                        }
                    });

                // 데이터 업데이트
                pointCloud.positions = newPositions;
                if (updateNormals)
                {
                    pointCloud.normals = newNormals;
                }
            }

            TE(SurfaceFitting);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud) return;

            size_t count = cachedPointCloud->numberOfElements;
            for (size_t i = 0; i < count; ++i)
            {
                VD::AddSphere(
                    "FittedPoints",
                    cachedPointCloud->positions[i],
                    Configuration::pointVisualizationRadius,
                    glm::vec4(cachedPointCloud->colors[i], 1.0f)
                );
            }
        }

        // Parameters
        inline void SetKNeighbors(int k) { kNeighbors = k; }
        inline void SetUpdateNormals(bool update) { updateNormals = update; }
        inline void SetPreserveMarks(bool preserve) { preserveMarks = preserve; }

    private:
        int kNeighbors = 32;
		int iteration = 10;
        bool updateNormals = true;
        bool preserveMarks = true;

        // 3x3 Symmetric Matrix Eigen Decomposition (Jacobi Method or similar approximation)
        // returns Sorted Eigenvalues (x=smallest, y=mid, z=largest) and corresponding Vectors (columns of mat3)
        void ComputeEigenDecomposition(const glm::mat3& cov, glm::vec3& outEvals, glm::mat3& outEvecs)
        {
            // 1. Compute Eigenvalues using Cardan's method (Closed form)
            double m = (cov[0][0] + cov[1][1] + cov[2][2]) / 3.0;
            double p = (glm::pow(cov[0][0] - m, 2.0) + glm::pow(cov[1][1] - m, 2.0) + glm::pow(cov[2][2] - m, 2.0) +
                2.0 * (glm::pow(cov[0][1], 2.0) + glm::pow(cov[0][2], 2.0) + glm::pow(cov[1][2], 2.0))) / 6.0;

            double q = glm::determinant(cov - glm::mat3(m)) / 2.0;
            double phi = 0.0;
            if (p > 1e-12) phi = glm::atan(glm::sqrt(std::max(0.0, p * p * p - q * q)), q) / 3.0;
            if (phi < 0) phi += 3.14159265358979323846 / 3.0;

            double eig1 = m + 2.0 * std::sqrt(p) * std::cos(phi);
            double eig2 = m + 2.0 * std::sqrt(p) * std::cos(phi + 2.0 * 3.14159265358979323846 / 3.0);
            double eig3 = 3.0 * m - eig1 - eig2;

            outEvals = glm::vec3((float)eig1, (float)eig2, (float)eig3);

            // Sort Eigenvalues (Smallest first for plane normal)
            // Indices mapping: 0, 1, 2
            int i0 = 0, i1 = 1, i2 = 2;
            if (outEvals[i0] > outEvals[i1]) std::swap(i0, i1);
            if (outEvals[i1] > outEvals[i2]) std::swap(i1, i2);
            if (outEvals[i0] > outEvals[i1]) std::swap(i0, i1);

            // Reorder values
            glm::vec3 sortedEvals;
            sortedEvals.x = outEvals[i0];
            sortedEvals.y = outEvals[i1];
            sortedEvals.z = outEvals[i2];
            outEvals = sortedEvals;

            // 2. Compute Eigenvectors
            // For 3x3 symmetric, we can compute eigenvectors by cross product of rows of (A - lambda*I)
            // We need the eigenvector for the SMALLEST eigenvalue (i0) for the normal.
            // But we compute all for completeness.

            auto computeVec = [&](float lambda) -> glm::vec3 {
                glm::mat3 A = cov - glm::mat3(lambda);

                glm::vec3 r0(A[0][0], A[0][1], A[0][2]);
                glm::vec3 r1(A[1][0], A[1][1], A[1][2]);
                glm::vec3 r2(A[2][0], A[2][1], A[2][2]);

                // Try cross products
                glm::vec3 v1 = glm::cross(r0, r1);
                glm::vec3 v2 = glm::cross(r1, r2);
                glm::vec3 v3 = glm::cross(r2, r0);

                float l1 = glm::dot(v1, v1);
                float l2 = glm::dot(v2, v2);
                float l3 = glm::dot(v3, v3);

                glm::vec3 maxV = v1;
                if (l2 > l1) maxV = v2;
                if (l3 > std::max(l1, l2)) maxV = v3;

                if (glm::length(maxV) > 1e-6f) return glm::normalize(maxV);

                // If degenerate (identity matrix like), return dominant axis approximation
                return glm::vec3(1, 0, 0); // Fallback
                };

            // Calculate vectors for sorted eigenvalues
            outEvecs[0] = computeVec(outEvals.x); // Smallest -> Normal
            outEvecs[1] = computeVec(outEvals.y);
            // Last one via cross product to ensure orthogonality
            outEvecs[2] = glm::normalize(glm::cross(outEvecs[0], outEvecs[1]));
        }
    };

    class OperatorPointDensity : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorPointDensity(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(PointDensity);

            if (pointCloud.numberOfElements == 0) return;

            // 1. Build Spatial Partitioning if needed
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numberOfPoints = pointCloud.numberOfElements;
            densities.resize(numberOfPoints);

            float searchRadius = spatialPartitioning->cellSize * searchRadiusScale;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numberOfPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // 2. Calculate Density (Parallel)
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = pointCloud.positions[i];
                    int neighborCount = 0;

                    int gx = (int)std::floor((p.x - spatialPartitioning->aabb.min.x) / spatialPartitioning->cellSize);
                    int gy = (int)std::floor((p.y - spatialPartitioning->aabb.min.y) / spatialPartitioning->cellSize);
                    int gz = (int)std::floor((p.z - spatialPartitioning->aabb.min.z) / spatialPartitioning->cellSize);

                    for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz)
                    {
                        for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy)
                        {
                            for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx)
                            {
                                uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
                                auto it = spatialPartitioning->voxelPointListHead.find(key);

                                if (it == spatialPartitioning->voxelPointListHead.end()) continue;

                                int currIdx = it->second;
                                while (currIdx != -1)
                                {
                                    if (currIdx != i)
                                    {
                                        if (glm::distance2(p, pointCloud.positions[currIdx]) <= searchRadiusSq)
                                        {
                                            neighborCount++;
                                        }
                                    }
                                    currIdx = spatialPartitioning->nextPoint[currIdx];
                                }
                            }
                        }
                    }

                    // Store raw neighbor count as density
                    densities[i] = (float)neighborCount;
                });

            // 3. Compute Min/Max for Normalization
            if (numberOfPoints > 0)
            {
                auto result = std::minmax_element(densities.begin(), densities.end());
                minDensity = *result.first;
                maxDensity = *result.second;
            }

            TE(PointDensity);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud || densities.empty()) return;

            size_t count = cachedPointCloud->numberOfElements;
            float range = maxDensity - minDensity;
            if (range < 0.0001f) range = 1.0f;

            for (size_t i = 0; i < count; ++i)
            {
                float val = (densities[i] - minDensity) / range; // Normalize 0.0 ~ 1.0

                glm::vec3 color;
                if (val < 0.5f)
                {
                    color = glm::mix(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), val * 2.0f);
                }
                else
                {
                    color = glm::mix(glm::vec3(0, 1, 0), glm::vec3(0, 0, 1), (val - 0.5f) * 2.0f);
                }

                VD::AddSphere(
                    "PointDensity",
                    cachedPointCloud->positions[i],
                    Configuration::pointVisualizationRadius * (2.0f - val),
                    glm::vec4(color, 1.0f)
                );
            }
        }

        inline float GetSearchRadiusScale() const { return searchRadiusScale; }
        inline void SetSearchRadiusScale(float scale) { searchRadiusScale = scale; }

		inline int GetNeighborSearchOffset() const { return neighborSearchOffset; }
		inline void SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; }

        inline float GetMinDensity() const { return minDensity; }
        inline float GetMaxDensity() const { return maxDensity; }

    private:
        std::vector<float> densities;
        int neighborSearchOffset = 1;
        float searchRadiusScale = 2.0f; // Radius relative to voxel size
        float minDensity = 0.0f;
        float maxDensity = 0.0f;
    };

    class OperatorNormalDivergence : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorNormalDivergence(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(NormalDivergence);

            if (pointCloud.numberOfElements == 0) return;

            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;

            normalDivergences.assign(numPoints, 0.0f);

            float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // Calculate Divergence of Normal Vector Field
            // div(n) approx sum( dot(n_j - n_i, p_j - p_i) / |p_j - p_i|^2 )
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = pointCloud.positions[i];
                    const glm::vec3& n_i = pointCloud.normals[i];

                    float divSum = 0.0f;
                    float weightSum = 0.0f;

                    int gx = (int)std::floor((p.x - spatialPartitioning->aabb.min.x) / spatialPartitioning->cellSize);
                    int gy = (int)std::floor((p.y - spatialPartitioning->aabb.min.y) / spatialPartitioning->cellSize);
                    int gz = (int)std::floor((p.z - spatialPartitioning->aabb.min.z) / spatialPartitioning->cellSize);

                    for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz)
                    {
                        for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy)
                        {
                            for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx)
                            {
                                uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
                                auto it = spatialPartitioning->voxelPointListHead.find(key);
                                if (it == spatialPartitioning->voxelPointListHead.end()) continue;

                                int curr = it->second;
                                while (curr != -1)
                                {
                                    if (curr != i)
                                    {
                                        glm::vec3 diff = pointCloud.positions[curr] - p;
                                        float distSq = glm::dot(diff, diff);

                                        if (distSq <= searchRadiusSq && distSq > 1e-8f)
                                        {
                                            float dist = std::sqrt(distSq);
                                            glm::vec3 dir = diff / dist;

                                            // Difference in normals
                                            glm::vec3 n_diff = pointCloud.normals[curr] - n_i;

                                            // Project normal difference onto the displacement vector
                                            // This estimates how much the normal changes along the direction of neighbors
                                            float dotVal = glm::dot(n_diff, dir);

                                            // Weight by inverse distance to prioritize closer neighbors
                                            float weight = 1.0f / dist;

                                            divSum += dotVal * weight;
                                            weightSum += weight;
                                        }
                                    }
                                    curr = spatialPartitioning->nextPoint[curr];
                                }
                            }
                        }
                    }

                    if (weightSum > 1e-6f)
                    {
                        normalDivergences[i] = divSum / weightSum;
                    }
                });

            TE(NormalDivergence);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud || normalDivergences.empty()) return;

            size_t count = cachedPointCloud->numberOfElements;

            for (size_t i = 0; i < count; ++i)
            {
                float val = normalDivergences[i];

                // Skip points with near-zero divergence for cleaner visualization
                //if (std::abs(val) < 0.001f)
                if (std::abs(val) < 0.5f)
                {
                    // Optionally render original color or skip
                    VD::AddSphere("NormalDivergence_Neutral", cachedPointCloud->positions[i], Configuration::pointVisualizationRadius, glm::vec4(cachedPointCloud->colors[i], 1.0f));
                    continue;
                }

                // Normalize value for visualization sensitivity
                float t = glm::clamp(val * visualizationScale + 0.5f, 0.0f, 1.0f);

                glm::vec3 color;
                if (t < 0.5f)
                {
                    // Negative Divergence (Normals converging -> Concave): Blue-ish
                    float localT = t * 2.0f;
                    color = glm::mix(glm::vec3(0, 0, 1), glm::vec3(0.5f, 0.5f, 0.5f), localT);

                    VD::AddSphere(
                        "NormalDivergence_Concave",
                        cachedPointCloud->positions[i],
                        Configuration::pointVisualizationRadius,
                        glm::vec4(color, 1.0f)
                    );
                }
                else
                {
                    // Positive Divergence (Normals spreading -> Convex): Red-ish
                    float localT = (t - 0.5f) * 2.0f;
                    color = glm::mix(glm::vec3(0.5f, 0.5f, 0.5f), glm::vec3(1, 0, 0), localT);

                    VD::AddSphere(
                        "NormalDivergence_Convex",
                        cachedPointCloud->positions[i],
                        Configuration::pointVisualizationRadius,
                        glm::vec4(color, 1.0f)
                    );
                }
            }
        }

        inline void SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; }
        inline void SetVisualizationScale(float scale) { visualizationScale = scale; }

    private:
        std::vector<float> normalDivergences;
        float searchRadiusMultiplier = 2.0f;
        int neighborSearchOffset = 1;
        float visualizationScale = 50.0f; // Scale factor for color mapping
    };

    template<typename FilterFunctor>
    class OperatorCustomFilter : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorCustomFilter(bool needToRebuildSpatialPartitioning = true)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            if (pointCloud.numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;
            TS(Filter);
            size_t writeIdx = 0;
            FilterFunctor filterFunctor;
            filterFunctor.filter = this;
            for (size_t readIdx = 0; readIdx < pointCloud.numberOfElements; ++readIdx)
            {
                if (filterFunctor(pointCloud, readIdx))
                {
                    if (writeIdx != readIdx)
                    {
                        pointCloud.positions[writeIdx] = pointCloud.positions[readIdx];
                        pointCloud.normals[writeIdx] = pointCloud.normals[readIdx];
                        pointCloud.colors[writeIdx] = pointCloud.colors[readIdx];
                        pointCloud.pointDeepLearningClassIDs[writeIdx] = pointCloud.pointDeepLearningClassIDs[readIdx];
                    }
                    writeIdx++;
                }
            }
            pointCloud.numberOfElements = writeIdx;
            pointCloud.positions.resize(writeIdx);
            pointCloud.normals.resize(writeIdx);
            pointCloud.colors.resize(writeIdx);
            pointCloud.pointDeepLearningClassIDs.resize(writeIdx);
            pointCloud.pointClusterIDs.resize(writeIdx);
            pointCloud.marks.resize(writeIdx);
            //alog("Filtered points. Remaining points: %s\n", FormatWithCommas(writeIdx).c_str());
            TE(Filter);
        }

        virtual void Visualize() override
        {
            auto numberOfPoints = cachedPointCloud->numberOfElements;
            for (size_t i = 0; i < numberOfPoints; i++)
            {
                auto& p = cachedPointCloud->positions[i];
                auto& n = glm::normalize(cachedPointCloud->normals[i]);
                auto& c = cachedPointCloud->colors[i];
                VD::AddSphere("FilteredPoints", p, n, Configuration::pointVisualizationRadius, glm::vec4(c, 1.0f));
            }
        }
    };

    class OperatorFilterETC : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorFilterETC(bool needToRebuildSpatialPartitioning = true)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
		}

        virtual void Process(PointCloud& pointCloud) override
        {
            if (pointCloud.numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = &pointCloud;

            TS(FilterETC);
            size_t writeIdx = 0;
            for (size_t readIdx = 0; readIdx < pointCloud.numberOfElements; ++readIdx)
            {
                int classID = pointCloud.pointDeepLearningClassIDs[readIdx];
                if (DL_ETC == classID)
                {
                    continue;
                }
                if (writeIdx != readIdx)
                {
                    pointCloud.positions[writeIdx] = pointCloud.positions[readIdx];
                    pointCloud.normals[writeIdx] = pointCloud.normals[readIdx];
                    pointCloud.colors[writeIdx] = pointCloud.colors[readIdx];
                    pointCloud.pointDeepLearningClassIDs[writeIdx] = pointCloud.pointDeepLearningClassIDs[readIdx];
                }
                writeIdx++;
            }
            pointCloud.numberOfElements = writeIdx;
            pointCloud.positions.resize(writeIdx);
            pointCloud.normals.resize(writeIdx);
			pointCloud.colors.resize(writeIdx);
            pointCloud.pointDeepLearningClassIDs.resize(writeIdx);
            pointCloud.pointClusterIDs.resize(writeIdx);
			pointCloud.marks.resize(writeIdx);
            //alog("Filtered ETC points. Remaining points: %s\n", FormatWithCommas(writeIdx).c_str());
			TE(FilterETC);
        }

        virtual void Visualize() override
        {
			auto numberOfPoints = cachedPointCloud->numberOfElements;

            for (size_t i = 0; i < numberOfPoints; i++)
            {
				auto& p = cachedPointCloud->positions[i];
				auto& n = glm::normalize(cachedPointCloud->normals[i]);
				auto& c = cachedPointCloud->colors[i];
				VD::AddSphere("FilteredPoints", p, n, Configuration::pointVisualizationRadius, glm::vec4(c, 1.0f));
            }
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

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(Clustering_Parallel);

            if (pointCloud.numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = &pointCloud;
            size_t numberOfPoints = pointCloud.numberOfElements;

            if (numberOfPoints != pointCloud.pointClusterIDs.size())
            {
                pointCloud.pointClusterIDs.resize(numberOfPoints, -1);
            }
            else
            {
				std::fill(pointCloud.pointClusterIDs.begin(), pointCloud.pointClusterIDs.end(), -1);
            }

            AtomicDisjointSet dsu;
            dsu.Initialize(numberOfPoints);

            float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
            float searchRadiusSq = searchRadius * searchRadius;

            float strictAngleThreshold = 0.9f;

            float planeDistThreshold = spatialPartitioning->cellSize * 0.2f;

            struct CellData { uint64_t key; int headIdx; };
            std::vector<CellData> flatCells;
            flatCells.reserve(spatialPartitioning->voxelPointListHead.size());

            for (const auto& pair : spatialPartitioning->voxelPointListHead)
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

                    for (int i = headIdx; i != -1; i = spatialPartitioning->nextPoint[i])
                    {
                        const glm::vec3& pA = pointCloud.positions[i];
                        const glm::vec3& nA = pointCloud.normals[i];

                        for (int j = spatialPartitioning->nextPoint[i]; j != -1; j = spatialPartitioning->nextPoint[j])
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

                                    uint64_t neighborKey = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
                                    if (neighborKey < key) continue; // 중복 방지

                                    auto it = spatialPartitioning->voxelPointListHead.find(neighborKey);
                                    if (it == spatialPartitioning->voxelPointListHead.end()) continue;

                                    int neighborHead = it->second;
                                    for (int j = neighborHead; j != -1; j = spatialPartitioning->nextPoint[j])
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
                pointCloud.pointClusterIDs[i] = rootToClusterID[root];
            }

            {
                std::unordered_map<int, int> rootSizeMap;
                for (size_t i = 0; i < numberOfPoints; ++i)
                {
                    int root = dsu.Find((int)i);
                    rootSizeMap[root]++;
                }

                sortedClusters.clear();
                sortedClusters.reserve(rootSizeMap.size());
                for (auto const& [root, size] : rootSizeMap)
                {
                    sortedClusters.emplace_back(root, size);
                }

                std::sort(sortedClusters.begin(), sortedClusters.end(),
                    [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                        return a.second > b.second;
                    });

                std::unordered_map<int, int> rootToSortedId;
                int currentClusterCount = 0;
                for (const auto& pair : sortedClusters)
                {
                    rootToSortedId[pair.first] = currentClusterCount++;
                }

                for (size_t i = 0; i < numberOfPoints; ++i)
                {
                    int root = dsu.Find((int)i);
                    pointCloud.pointClusterIDs[i] = rootToSortedId[root];
                }

                alog("Clustering Done. Found %d clusters.\n", currentClusterCount);
                if (sortedClusters.size() > 0) alog(" - Biggest(ID 0): %d points\n", sortedClusters[0].second);
                if (sortedClusters.size() > 1) alog(" - 2nd(ID 1): %d points\n", sortedClusters[1].second);
                if (sortedClusters.size() > 2) alog(" - 3rd(ID 2): %d points\n", sortedClusters[2].second);
            }

            TE(Clustering_Parallel);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud || cachedPointCloud->pointClusterIDs.empty()) return;

            auto contrastingColors = Color::GetContrastingColorsWithoutBWRGB(256);

            size_t count = cachedPointCloud->numberOfElements;
            for (size_t i = 0; i < count; ++i)
            {
                int clusterId = cachedPointCloud->pointClusterIDs[i];
                if (clusterId < 0) continue;

                const glm::vec3& p = cachedPointCloud->positions[i];

                //auto mark = cachedPointCloud->marks[i];
    //            if(1 == mark)
    //            {
    //                VD::AddSphere(
    //                    "ClusteredPoints_Mark1",
    //                    p,
    //                    Configuration::pointVisualizationRadius * 1.2f,
    //                    glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
    //                );
    //            }
    //            else
    //            {
    //                VD::AddSphere(
    //                    "ClusteredPoints",
    //                    p,
    //                    Configuration::pointVisualizationRadius,
    //                    contrastingColors[clusterId % contrastingColors.size()]
    //                );
    //            }

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

        inline const std::vector<std::pair<int, int>>& GetSortedClusters() const { return sortedClusters; }

    protected:
        float searchRadiusMultiplier = 1.5f;
		bool useMarksForClustering = false;

        std::vector<std::pair<int, int>> sortedClusters;
    };

    class OperatorClusterBorderFinding : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorClusterBorderFinding(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(ClusterBorderFinding);
            if (pointCloud.numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = &pointCloud;
            size_t numberOfPoints = pointCloud.numberOfElements;
            borderPointIndices.clear();
            float searchRadius = spatialPartitioning->cellSize * 1.5f;
            float searchRadiusSq = searchRadius * searchRadius;
            struct CellData { uint64_t key; int headIdx; };
            std::vector<CellData> flatCells;
            flatCells.reserve(spatialPartitioning->voxelPointListHead.size());
            for (const auto& pair : spatialPartitioning->voxelPointListHead)
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
                    for (int i = headIdx; i != -1; i = spatialPartitioning->nextPoint[i])
                    {
                        const glm::vec3& pA = pointCloud.positions[i];
                        int clusterA = pointCloud.pointClusterIDs[i];
                        bool isBorder = false;
                        for (int dz = -1; dz <= 1 && !isBorder; ++dz)
                        {
                            for (int dy = -1; dy <= 1 && !isBorder; ++dy)
                            {
                                for (int dx = -1; dx <= 1 && !isBorder; ++dx)
                                {
                                    if (dx == 0 && dy == 0 && dz == 0) continue;
                                    uint64_t neighborKey = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
                                    auto it = spatialPartitioning->voxelPointListHead.find(neighborKey);
                                    if (it == spatialPartitioning->voxelPointListHead.end()) continue;
                                    int neighborHead = it->second;
                                    for (int j = neighborHead; j != -1; j = spatialPartitioning->nextPoint[j])
                                    {
                                        const glm::vec3& pB = pointCloud.positions[j];
                                        if (glm::distance2(pA, pB) > searchRadiusSq) continue;
                                        int clusterB = pointCloud.pointClusterIDs[j];
                                        //if (clusterA != clusterB)
                                        if(0 != clusterA && 0 == clusterB)
                                        {
                                            isBorder = true;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        if (isBorder)
                        {
                            std::lock_guard<std::mutex> lock(borderIndicesMutex);
                            borderPointIndices.push_back(i);
                        }
                    }
                });
            alog("Cluster Border Finding Done. Found %d border points.\n", (int)borderPointIndices.size());
            TE(ClusterBorderFinding);
        }
        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud) return;

            cachedPointCloud->marks.clear();
            cachedPointCloud->marks.resize(cachedPointCloud->positions.size());

            for (const auto& idx : borderPointIndices)
            {
                cachedPointCloud->marks[idx] = 1;
            }

            for (size_t i = 0; i < cachedPointCloud->numberOfElements; i++)
            {
				auto& p = cachedPointCloud->positions[i];
				auto& n = glm::normalize(cachedPointCloud->normals[i]);
				auto& c = cachedPointCloud->colors[i];
				auto& mark = cachedPointCloud->marks[i];

                if(1 == mark)
                {
                    VD::AddSphere(
                        "BorderPoints_Mark1",
                        p,
                        n,
                        Configuration::pointVisualizationRadius * 1.2f,
                        glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)
                    );
				}
                else
                {
                    VD::AddSphere("AllPoints", p, n, Configuration::pointVisualizationRadius, glm::vec4(c, 1.0f));
                }
            }
        }
    protected:
        std::vector<int> borderPointIndices;
        std::mutex borderIndicesMutex;
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

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(ComplexClustering);

            if (pointCloud.numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;
            pointClusterIds.assign(numPoints, -1);
            pointCurvatures.resize(numPoints);

            // DL Class ID 데이터 유효성 확인
            bool hasValidClassIDs = params.useDeepLearningClasses &&
                !pointCloud.pointDeepLearningClassIDs.empty() &&
                (pointCloud.pointDeepLearningClassIDs.size() == numPoints);

            // ----------------------------------------------------------------
            // Phase 1: 곡률(Curvature) 및 로컬 특징 사전 계산 (병렬)
            // ----------------------------------------------------------------
            TS(PrecalcFeatures);
            {
                std::vector<int> indices(numPoints);
                std::iota(indices.begin(), indices.end(), 0);
                float curvSearchRadSq = std::pow(spatialPartitioning->cellSize * 2.0f, 2.0f);

                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                    {
                        glm::vec3 center = pointCloud.positions[i];
                        int neighbors = 0;

                        int gx = (int)std::floor((center.x - spatialPartitioning->aabb.min.x) / spatialPartitioning->cellSize);
                        int gy = (int)std::floor((center.y - spatialPartitioning->aabb.min.y) / spatialPartitioning->cellSize);
                        int gz = (int)std::floor((center.z - spatialPartitioning->aabb.min.z) / spatialPartitioning->cellSize);

                        float sumDistSq = 0.0f;

                        for (int dz = -1; dz <= 1; ++dz) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dx = -1; dx <= 1; ++dx) {
                                    uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
                                    auto it = spatialPartitioning->voxelPointListHead.find(key);
                                    if (it == spatialPartitioning->voxelPointListHead.end()) continue;

                                    for (int idx = it->second; idx != -1; idx = spatialPartitioning->nextPoint[idx]) {
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

            float searchRadiusSq = std::pow(spatialPartitioning->cellSize * params.searchRadiusMult, 2.0f);
            float planeDistAbs = spatialPartitioning->cellSize * params.planeOffsetThreshold;

            struct CellData { uint64_t key; int headIdx; };
            std::vector<CellData> flatCells;
            flatCells.reserve(spatialPartitioning->voxelPointListHead.size());
            for (const auto& pair : spatialPartitioning->voxelPointListHead) flatCells.push_back({ pair.first, pair.second });

            const uint64_t mask = 0x1FFFFF;

            std::for_each(std::execution::par, flatCells.begin(), flatCells.end(), [&](const CellData& cell)
                {
                    uint64_t key = cell.key;
                    int gx = (int)(key >> 42);
                    int gy = (int)((key >> 21) & mask);
                    int gz = (int)(key & mask);

                    for (int i = cell.headIdx; i != -1; i = spatialPartitioning->nextPoint[i])
                    {
                        const glm::vec3& pA = pointCloud.positions[i];
                        const glm::vec3& nA = pointCloud.normals[i];
                        const glm::vec3& cA = pointCloud.colors[i];
                        float curvA = pointCurvatures[i];
                        int classA = hasValidClassIDs ? pointCloud.pointDeepLearningClassIDs[i] : -1;

                        auto CheckAndMerge = [&](int j)
                            {
                                // 0. Class ID Check (Hard Constraint) [NEW]
                                // 서로 다른 클래스(예: 치아 vs 잇몸)라면 기하학적으로 가까워도 무조건 분리
                                if (hasValidClassIDs)
                                {
                                    if (classA != pointCloud.pointDeepLearningClassIDs[j]) return;
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
                        for (int j = spatialPartitioning->nextPoint[i]; j != -1; j = spatialPartitioning->nextPoint[j]) {
                            CheckAndMerge(j);
                        }

                        // (B) Inter-Cell
                        for (int dz = -1; dz <= 1; ++dz) {
                            for (int dy = -1; dy <= 1; ++dy) {
                                for (int dx = -1; dx <= 1; ++dx) {
                                    if (dx == 0 && dy == 0 && dz == 0) continue;
                                    uint64_t nKey = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
                                    if (nKey < key) continue;

                                    auto it = spatialPartitioning->voxelPointListHead.find(nKey);
                                    if (it == spatialPartitioning->voxelPointListHead.end()) continue;

                                    for (int j = it->second; j != -1; j = spatialPartitioning->nextPoint[j]) {
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

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(Curvature_Parallel);

            if (pointCloud.numberOfElements == 0) return;

            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = &pointCloud;
            size_t numberOfPoints = pointCloud.numberOfElements;
            curvatures.resize(numberOfPoints);

            pointCloud.marks.clear();
			pointCloud.marks.resize(numberOfPoints, 0); // Reset Marks

            float searchRadius = spatialPartitioning->cellSize * searchRadiusScale;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numberOfPoints);
            std::iota(indices.begin(), indices.end(), 0);

            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = pointCloud.positions[i];

                    std::vector<int> neighbors;
                    neighbors.reserve(64);

                    glm::vec3 centroid(0.0f);

                    int gx = (int)std::floor((p.x - spatialPartitioning->aabb.min.x) / spatialPartitioning->cellSize);
                    int gy = (int)std::floor((p.y - spatialPartitioning->aabb.min.y) / spatialPartitioning->cellSize);
                    int gz = (int)std::floor((p.z - spatialPartitioning->aabb.min.z) / spatialPartitioning->cellSize);

                    for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz)
                    {
                        for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy)
                        {
                            for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx)
                            {
                                uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
                                auto it = spatialPartitioning->voxelPointListHead.find(key);
                                if (it == spatialPartitioning->voxelPointListHead.end()) continue;

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
                                    currIdx = spatialPartitioning->nextPoint[currIdx];
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

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(CurvatureDivergence_Total);

            if (pointCloud.numberOfElements == 0) return;

            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;

            curvatures.assign(numPoints, 0.0f);
            gradients.assign(numPoints, glm::vec3(0.0f));
            divergences.assign(numPoints, 0.0f);

            float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
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

                    ProcessNeighbors(i, p, *spatialPartitioning, searchRadiusSq, [&](int neighborIdx) {
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

                    ProcessNeighbors(i, p, *spatialPartitioning, searchRadiusSq, [&](int j) {
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

                    ProcessNeighbors(i, p, *spatialPartitioning, searchRadiusSq, [&](int j) {
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
                        auto it = sg.voxelPointListHead.find(key);
                        if (it == sg.voxelPointListHead.end()) continue;

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

    class OperatorMeshGeneration : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorMeshGeneration(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(MeshGeneration);

            if (pointCloud.numberOfElements == 0) return;

            // 1. Build Spatial Partitioning if needed (used for AABB mainly here)
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(pointCloud, Configuration::voxelSize);
                needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = &pointCloud;

            // 2. Initialize SparseDataBlock
            // Clear previous data to ensure clean generation
            sparseDataBlock.dataBlocks.clear();
            sparseDataBlock.voxelSize = meshVoxelSize;

            // 3. Populate Voxel Data (Implicit Surface Generation)
            // Converts explicit point cloud into implicit signed distance field
            sparseDataBlock.FromPointsData(
                pointCloud.positions,
                pointCloud.normals,
                pointCloud.colors,
                pointCloud.pointClusterIDs,
                spatialPartitioning->aabb.min
            );

            // 4. Generate Mesh
            // Extracts surface geometry from the implicit field
            meshGenerator.Generate(sparseDataBlock);

            if (false == exportFilename.empty())
            {
				meshGenerator.ExportPLY(exportFilename);
            }

            // 5. Post-processing
            if (detectHoles)
            {
                meshGenerator.DetectHoles();
            }

            // Optional: Log generation result
            // printf("Mesh Generated. Triangles: %zu\n", meshGenerator.triangles.size());

            TE(MeshGeneration);
        }

        virtual void Visualize() override
        {
            // Use MeshGenerator's internal visualization logic
            meshGenerator.Visualize(showMesh, showHoles);
        }

        void ExportPLY(const std::string& filename)
        {
            exportFilename = filename;
        }

		inline float GetMeshVoxelSize() const { return meshVoxelSize; }
		inline void SetMeshVoxelSize(float size) { meshVoxelSize = size; }

        inline void SetShowMesh(bool show) { showMesh = show; }
        inline void SetShowHoles(bool show) { showHoles = show; }
        inline void SetDetectHoles(bool detect) { detectHoles = detect; }

        inline std::vector<Triangle>& GetTriangles() { return meshGenerator.triangles; }
        inline const std::vector<Triangle>& GetTriangles() const { return meshGenerator.triangles; }

    private:
        float meshVoxelSize = Configuration::voxelSize;

        SparseDataBlock sparseDataBlock;
        MeshGenerator meshGenerator;

        std::string exportFilename;

        bool showMesh = true;
        bool showHoles = true;
        bool detectHoles = true;
    };

    class OperatorMeshDistanceFilter : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorMeshDistanceFilter(bool needToRebuildSpatialPartitioning = false)
            : IGeometricProcessingOperator<SparseGrid>(needToRebuildSpatialPartitioning)
        {
        }

        void SetReferenceMesh(const std::vector<Triangle>& meshTriangles)
        {
            // [Fix] 메쉬 등록 시 퇴화 삼각형(면적이 거의 없는 삼각형)을 미리 걸러냄
            referenceMesh.clear();
            referenceMesh.reserve(meshTriangles.size());

            for (const auto& tri : meshTriangles)
            {
                glm::vec3 e1 = tri.v[1] - tri.v[0];
                glm::vec3 e2 = tri.v[2] - tri.v[0];
                glm::vec3 crossP = glm::cross(e1, e2);

                // 면적이 너무 작으면 거리 계산 시 Det가 0이 되어 NaN 유발 가능 -> 제외
                if (glm::length2(crossP) > 1e-12f)
                {
                    referenceMesh.push_back(tri);
                }
            }
        }

        virtual void Process(PointCloud& pointCloud) override
        {
            TS(MeshDistanceFilter);

            if (pointCloud.numberOfElements == 0 || referenceMesh.empty()) return;

            cachedPointCloud = &pointCloud;
            size_t numPoints = pointCloud.numberOfElements;

            distances.resize(numPoints);
            pointCloud.marks.assign(numPoints, 0);

            // 1. Build Spatial Grid for Triangles
            BuildTriangleGrid();

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // 2. Compute Distances (Parallel)
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    float d = GetClosestDistanceFromMesh(pointCloud.positions[i]);

                    if (std::isnan(d) || std::isinf(d)) d = FLT_MAX;

                    distances[i] = d;
                });

            int markedCount = 0;
            for (size_t i = 0; i < numPoints; ++i)
            {
                //if (distances[i] > averageDistance * thresholdMultiplier)
                //{
                //    pointCloud.marks[i] = 1;
                //    markedCount++;
                //}

                if (distances[i] > Configuration::voxelSize * thresholdMultiplier)
                {
                    pointCloud.marks[i] = 1;
                    markedCount++;
                }
            }

            // 로그 출력 (디버깅용)
            // printf("Mesh Distance Filter: Avg Dist = %.4f, Marked Points = %d\n", averageDistance, markedCount);

            TE(MeshDistanceFilter);
        }

        virtual void Visualize() override
        {
            if (nullptr == cachedPointCloud || distances.empty()) return;

            size_t count = cachedPointCloud->numberOfElements;

            for (size_t i = 0; i < count; ++i)
            {
                if (cachedPointCloud->marks[i] == 1)
                {
                    VD::AddSphere(
                        "HighDistancePoints",
                        cachedPointCloud->positions[i],
                        Configuration::pointVisualizationRadius * 1.2f,
                        glm::vec4(1.0f, 0.0f, 0.0f, 1.0f) // Red
                    );
                }
                else
                {
                    float t = (averageDistance > 1e-6f) ? glm::clamp(distances[i] / (averageDistance * 2.0f), 0.0f, 1.0f) : 0.0f;
                    glm::vec3 color = glm::mix(glm::vec3(0, 0, 1), glm::vec3(0, 1, 1), t);

                    VD::AddSphere(
                        "NormalDistancePoints",
                        cachedPointCloud->positions[i],
                        Configuration::pointVisualizationRadius,
                        glm::vec4(color, 0.5f)
                    );
                }
            }
        }

        inline void SetThresholdMultiplier(float mult) { thresholdMultiplier = mult; }
        inline float GetAverageDistance() const { return averageDistance; }

    private:
        std::vector<Triangle> referenceMesh;
        std::vector<float> distances;
        float averageDistance = 0.0f;
        float thresholdMultiplier = 3.0f;

        struct TriGridKey
        {
            int x, y, z;
            bool operator==(const TriGridKey& o) const { return x == o.x && y == o.y && z == o.z; }
        };
        struct TriGridHash
        {
            size_t operator()(const TriGridKey& k) const {
                return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) << 1);
            }
        };

        robin_hood::unordered_flat_map<TriGridKey, std::vector<int>, TriGridHash> triangleGrid;
        float triGridSize = 0.0f;
        glm::vec3 triGridMin = glm::vec3(0.0f);

        void BuildTriangleGrid()
        {
            triangleGrid.clear();
            if (referenceMesh.empty()) return;

            AABB meshAABB;
            for (const auto& tri : referenceMesh)
            {
                meshAABB.Expand(tri.v[0]);
                meshAABB.Expand(tri.v[1]);
                meshAABB.Expand(tri.v[2]);
            }
            triGridMin = meshAABB.min - glm::vec3(0.1f);
            triGridSize = Configuration::voxelSize * 5.0f;

            for (int i = 0; i < (int)referenceMesh.size(); ++i)
            {
                const auto& tri = referenceMesh[i];

                glm::vec3 tMin = glm::min(glm::min(tri.v[0], tri.v[1]), tri.v[2]);
                glm::vec3 tMax = glm::max(glm::max(tri.v[0], tri.v[1]), tri.v[2]);

                int minX = (int)std::floor((tMin.x - triGridMin.x) / triGridSize);
                int minY = (int)std::floor((tMin.y - triGridMin.y) / triGridSize);
                int minZ = (int)std::floor((tMin.z - triGridMin.z) / triGridSize);

                int maxX = (int)std::floor((tMax.x - triGridMin.x) / triGridSize);
                int maxY = (int)std::floor((tMax.y - triGridMin.y) / triGridSize);
                int maxZ = (int)std::floor((tMax.z - triGridMin.z) / triGridSize);

                for (int z = minZ; z <= maxZ; ++z)
                {
                    for (int y = minY; y <= maxY; ++y)
                    {
                        for (int x = minX; x <= maxX; ++x)
                        {
                            triangleGrid[{x, y, z}].push_back(i);
                        }
                    }
                }
            }
        }

        float GetClosestDistanceFromMesh(const glm::vec3& p)
        {
            float minDistSq = FLT_MAX;

            int gx = (int)std::floor((p.x - triGridMin.x) / triGridSize);
            int gy = (int)std::floor((p.y - triGridMin.y) / triGridSize);
            int gz = (int)std::floor((p.z - triGridMin.z) / triGridSize);

            bool found = false;

            for (int r = 0; r <= 2; ++r)
            {
                for (int dz = -r; dz <= r; ++dz)
                {
                    for (int dy = -r; dy <= r; ++dy)
                    {
                        for (int dx = -r; dx <= r; ++dx)
                        {
                            auto it = triangleGrid.find({ gx + dx, gy + dy, gz + dz });
                            if (it != triangleGrid.end())
                            {
                                for (int triIdx : it->second)
                                {
                                    float sq = SqDistPointTriangle(p, referenceMesh[triIdx]);
                                    // [Fix] sq가 NaN인 경우 무시
                                    if (!std::isnan(sq) && sq < minDistSq)
                                    {
                                        minDistSq = sq;
                                        found = true;
                                    }
                                }
                            }
                        }
                    }
                }
                if (found && minDistSq < (triGridSize * r * triGridSize * r)) break;
            }

            if (!found) return 1000.0f;

            return std::sqrt(minDistSq);
        }

        // [Fix] Point - Triangle Squared Distance Helper (Robust Version)
        float SqDistPointTriangle(const glm::vec3& p, const Triangle& tri)
        {
            glm::vec3 B = tri.v[0];
            glm::vec3 E0 = tri.v[1] - B;
            glm::vec3 E1 = tri.v[2] - B;
            glm::vec3 D = B - p;
            float a = glm::dot(E0, E0);
            float b = glm::dot(E0, E1);
            float c = glm::dot(E1, E1);
            float d = glm::dot(E0, D);
            float e = glm::dot(E1, D);
            float f = glm::dot(D, D);

            float det = a * c - b * b;
            float s = b * e - c * d;
            float t = b * d - a * e;

            // [Fix] Determinant가 0에 가까우면(Degenerate Triangle) 안전하게 처리
            if (std::abs(det) < 1e-12f)
            {
                // 삼각형이 선분이나 점으로 퇴화된 경우, 가장 가까운 꼭짓점과의 거리 반환
                float d0 = glm::distance2(p, tri.v[0]);
                float d1 = glm::distance2(p, tri.v[1]);
                float d2 = glm::distance2(p, tri.v[2]);
                return std::min({ d0, d1, d2 });
            }

            if (s + t <= det)
            {
                if (s < 0.f)
                {
                    if (t < 0.f)  // region 4
                    {
                        if (d < 0.f) { t = 0.f; if (-d >= a) { s = 1.f; } else { s = -d / a; } }
                        else { s = 0.f; if (e >= 0.f) { t = 0.f; } else if (-e >= c) { t = 1.f; } else { t = -e / c; } }
                    }
                    else  // region 3
                    {
                        s = 0.f; if (e >= 0.f) { t = 0.f; }
                        else if (-e >= c) { t = 1.f; }
                        else { t = -e / c; }
                    }
                }
                else if (t < 0.f)  // region 5
                {
                    t = 0.f; if (d >= 0.f) { s = 0.f; }
                    else if (-d >= a) { s = 1.f; }
                    else { s = -d / a; }
                }
                else  // region 0
                {
                    float invDet = 1.f / det; s *= invDet; t *= invDet;
                }
            }
            else
            {
                if (s < 0.f)  // region 2
                {
                    float tmp0 = b + d; float tmp1 = c + e;
                    if (tmp1 > tmp0) { float numer = tmp1 - tmp0; float denom = a - 2.f * b + c; s = (numer >= denom) ? 1.f : numer / denom; t = 1.f - s; }
                    else { s = 0.f; if (tmp1 <= 0.f) { t = 1.f; } else if (e >= 0.f) { t = 0.f; } else { t = -e / c; } }
                }
                else if (t < 0.f)  // region 6
                {
                    float tmp0 = b + e; float tmp1 = a + d;
                    if (tmp1 > tmp0) { float numer = tmp1 - tmp0; float denom = a - 2.f * b + c; t = (numer >= denom) ? 1.f : numer / denom; s = 1.f - t; }
                    else { t = 0.f; if (tmp1 <= 0.f) { s = 1.f; } else if (d >= 0.f) { s = 0.f; } else { s = -d / a; } }
                }
                else  // region 1
                {
                    float numer = c + e - b - d; float denom = a - 2.f * b + c;
                    if (numer <= 0.f) { s = 0.f; }
                    else if (numer >= denom) { s = 1.f; }
                    else { s = numer / denom; }
                    t = 1.f - s;
                }
            }
            return a * s * s + 2.f * b * s * t + c * t * t + 2.f * d * s + 2.f * e * t + f;
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

                    auto operatorMeshDistanceFilter = std::dynamic_pointer_cast<OperatorMeshDistanceFilter>(op);
                    if (operatorMeshDistanceFilter)
                    {
						operatorMeshDistanceFilter->SetReferenceMesh(generatedMeshTriangles);
                    }

                    op->Process(pointCloud, sparseGrid);

                    auto operatorMeshGeneration = std::dynamic_pointer_cast<OperatorMeshGeneration>(op);
                    if (operatorMeshGeneration)
                    {
						operatorMeshGeneration->GetTriangles().swap(generatedMeshTriangles);
                    }

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

        void VisualizeLast()
        {
            if (!operators.empty())
            {
                auto& [tag, op] = operators.back();
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
		std::vector<Triangle> generatedMeshTriangles;
    };
}
