#pragma once

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
    class Pipeline;

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

        void Resize(size_t newSize)
        {
            numberOfElements = newSize;
            positions.resize(newSize);
			normals.resize(newSize);
			colors.resize(newSize);
			pointDeepLearningClassIDs.resize(newSize, -1);
            pointClusterIDs.resize(newSize, -1);
			marks.resize(newSize, -1);
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

        void CopyFrom(const PointCloud& src)
        {
            Clear();

            numberOfElements = src.numberOfElements;
            positions = src.positions;
            normals = src.normals;
            colors = src.colors;
            pointDeepLearningClassIDs = src.pointDeepLearningClassIDs;
            pointClusterIDs = src.pointClusterIDs;
            marks = src.marks;
            aabb = src.aabb;
		}

        void CopyTo(PointCloud& dst) const
        {
            dst.Clear();

            dst.numberOfElements = numberOfElements;
            dst.positions = positions;
            dst.normals = normals;
            dst.colors = colors;
            dst.pointDeepLearningClassIDs = pointDeepLearningClassIDs;
            dst.pointClusterIDs = pointClusterIDs;
            dst.marks = marks;
			dst.aabb = aabb;
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
            if (pc.numberOfElements == 0)
            {
				aerr("PointCloud is empty. Cannot build SparseGrid.\n");
                return;
            }

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
            result.distance = FLT_MAX; // [중요 1] 초기화: 무한대 거리
            result.hasHit = false;

            if (points.empty() || voxelPointListHead.empty()) return result;

            // 1. AABB 교차 검사
            float tEntry = 0.0f, tExit = 0.0f;
            if (!aabb.IntersectRay(ray, tEntry, tExit)) return result;

            // Ray가 박스 내부에서 시작하는 경우 tEntry 보정
            if (tEntry < 0.0f) tEntry = 0.0f;

            // 2. 시작 위치 보정 (부동소수점 오차로 인한 경계면 문제 방지)
            glm::vec3 startPos = ray.origin + ray.direction * (tEntry + 0.001f);

            // 3. 시작 복셀 인덱스 계산
            int curGx = (int)std::floor((startPos.x - aabb.min.x) / cellSize);
            int curGy = (int)std::floor((startPos.y - aabb.min.y) / cellSize);
            int curGz = (int)std::floor((startPos.z - aabb.min.z) / cellSize);

            // [중요 2] 인덱스 안전 장치 (음수가 나오면 해시 키 오류 발생)
            if (curGx < 0) curGx = 0;
            if (curGy < 0) curGy = 0;
            if (curGz < 0) curGz = 0;

            // 4. DDA 준비
            int stepX = (ray.direction.x >= 0) ? 1 : -1;
            int stepY = (ray.direction.y >= 0) ? 1 : -1;
            int stepZ = (ray.direction.z >= 0) ? 1 : -1;

            // 다음 경계까지의 거리 계산
            float nextBoundX = aabb.min.x + (curGx + (stepX > 0 ? 1 : 0)) * cellSize;
            float nextBoundY = aabb.min.y + (curGy + (stepY > 0 ? 1 : 0)) * cellSize;
            float nextBoundZ = aabb.min.z + (curGz + (stepZ > 0 ? 1 : 0)) * cellSize;

            float tMaxX = (nextBoundX - ray.origin.x) * ray.inverseDirection.x;
            float tMaxY = (nextBoundY - ray.origin.y) * ray.inverseDirection.y;
            float tMaxZ = (nextBoundZ - ray.origin.z) * ray.inverseDirection.z;

            float tDeltaX = std::abs(cellSize * ray.inverseDirection.x);
            float tDeltaY = std::abs(cellSize * ray.inverseDirection.y);
            float tDeltaZ = std::abs(cellSize * ray.inverseDirection.z);

            // 5. 복셀 순회
            // tExit보다 약간 더 여유 있게 순회
            while (tEntry <= tExit + cellSize * 0.5f)
            {
                uint64_t key = GetKey(curGx, curGy, curGz);
                auto it = voxelPointListHead.find(key);

                if (it != voxelPointListHead.end())
                {
                    int currIdx = it->second;

                    while (currIdx != -1)
                    {
                        float t = 0.0f;
                        if (ray.IntersectSphere(points[currIdx], pickRadius, t))
                        {
                            // [핵심 수정] 기존에 찾은 거리보다 "더 가까운 경우에만" 업데이트
                            if (t < result.distance)
                            {
                                result.hasHit = true;
                                result.pointIndex = currIdx;
                                result.distance = t;
                                result.gx = curGx; result.gy = curGy; result.gz = curGz;
                            }
                        }
                        currIdx = nextPoint[currIdx];
                    }

                    // [최적화] 현재 복셀 영역 내에서 가장 가까운 히트를 찾았다면 조기 종료
                    // 다음 복셀 경계(tNextBoundary)보다 현재 찾은 히트 거리(result.distance)가 짧다면
                    // 더 멀리 있는 복셀을 탐색할 필요가 없음.
                    float tNextBoundary = std::min(std::min(tMaxX, tMaxY), tMaxZ);
                    if (result.hasHit && result.distance <= tNextBoundary)
                    {
                        return result;
                    }
                }

                // DDA Step (다음 복셀로 이동)
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
                auto entity = Feather.CreateEntity("Mesh");
                auto renderable = Feather.CreateComponent<Renderable>(entity);
                renderable->Initialize(Renderable::GeometryMode::Triangles);
                renderable->AddShader(Feather.CreateShader("Default", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
                renderable->AddShader(Feather.CreateShader("TwoSide", File("../../res/Shaders/TwoSide.vs"), File("../../res/Shaders/TwoSide.fs")));
                renderable->SetActiveShaderIndex(0);
                renderable->AddVertices(v); renderable->AddNormals(n); renderable->AddColors(c); renderable->AddIndices(ind);

                Feather.CreateEventCallback<KeyEvent>(entity, [](Entity e, const KeyEvent& event) {
                    if (event.action == 0 && event.keyCode == GLFW_KEY_GRAVE_ACCENT) Feather.GetComponent<Renderable>(e)->NextDrawingMode();
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

    struct GeometricProcessingOperatorParameter
    {
		std::map<std::string, std::any> parameters;

		template<typename T>
        void SetParameter(const std::string& name, const T& value)
        {
            parameters[name] = value;
        }

		template<typename T>
        T GetParameter(const std::string& name, const T& defaultValue) const
        {
            auto it = parameters.find(name);
            if (it != parameters.end())
            {
                try
                {
                    return std::any_cast<T>(it->second);
                }
                catch (const std::bad_any_cast&)
                {
                    return defaultValue;
                }
            }
            return defaultValue;
		}

        bool needToDeleteSpatialPartitioning = false;
        bool needToRebuildSpatialPartitioning = false;
        bool needToStorePointCloud = false;
    };

    class IGeometricProcessingOperatorBase {};

	template<typename SpatialPartitioningType>
	class IGeometricProcessingOperator : public IGeometricProcessingOperatorBase
    {
    public:
		IGeometricProcessingOperator(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : pipeline(pipeline), parameter(parameter) {}
		virtual ~IGeometricProcessingOperator()
        {
            if (parameter.needToDeleteSpatialPartitioning)
            {
                SAFE_DELETE(spatialPartitioning);
            }
        }

        virtual void Process(PointCloud* currentPointCloud) = 0;
        void Process(PointCloud* currentPointCloud, SpatialPartitioningType* spatialPartitioning)
        {
			this->spatialPartitioning = spatialPartitioning;
			Process(currentPointCloud);
        }

		virtual void Visualize() = 0;

		inline SpatialPartitioningType* GetSpatialPartitioning() const { return spatialPartitioning; }

		inline GeometricProcessingOperatorParameter& GetParameter() { return parameter; }
		inline void SetGeometricProcessingOperatorParameter(const GeometricProcessingOperatorParameter& param) { parameter = param; }
        inline const std::vector<int>& GetPointTags() const { return pointTags; }
		inline void SetPointTags(const std::vector<int>& tags) { pointTags = tags; }

    protected:
		Pipeline* pipeline = nullptr;
        GeometricProcessingOperatorParameter parameter;
        SpatialPartitioningType* spatialPartitioning = nullptr;
        std::vector<int> pointTags;
		PointCloud* cachedPointCloud = nullptr;
	};

    class Pipeline
    {
    public:
        Pipeline() = default;
        ~Pipeline();

        void BuildSparseGrid(PointCloud& pointCloud);

        template<typename OperatorType>
        std::shared_ptr<OperatorType> AddOperator(const std::string& tag, const GeometricProcessingOperatorParameter& parameter)
        {
            auto op = std::make_shared<OperatorType>(this, parameter);
            operators.emplace_back(std::make_tuple(tag, op));
            return op;
        }

        void Execute();
        void VisualizeAll();
        void VisualizeLast();
        void Clear();

        void CreatePointCloud();
		void StorePointCloud();
        void RestoreInitialPointCloud();
        void RestoreLastPointCloud();

        inline SparseGrid* GetSparseGrid() const { return sparseGrid; }

		inline PointCloud* GetCurrentPointCloud() const { return currentPointCloud; }
		inline PointCloud* GetLastPointCloud() { return (pointClouds.size() >= 2) ? &pointClouds[pointClouds.size() - 2] : nullptr; }

    protected:
        std::vector<std::tuple<std::string, std::shared_ptr<IGeometricProcessingOperator<SparseGrid>>>> operators;
        SparseGrid* sparseGrid = nullptr;
        std::vector<Triangle> generatedMeshTriangles;

        std::vector<PointCloud> pointClouds;
        PointCloud* currentPointCloud = nullptr;
    };
}
