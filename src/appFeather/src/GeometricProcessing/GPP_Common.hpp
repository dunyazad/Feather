#pragma once

#include <robin_hood.h>
#include <libFeather.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <execution>
#include <mutex>
#include <memory>
#include <queue>
#include <tuple>
#include <any>

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
    DL_GINGIVA3,
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

    struct Ray
    {
        Eigen::Vector3f origin;
        Eigen::Vector3f direction;
        Eigen::Vector3f inverseDirection;

        Ray(const Eigen::Vector3f& o, const Eigen::Vector3f& d) : origin(o), direction(d) {
            const float epsilon = 1e-6f;

            inverseDirection.x() = (std::abs(direction.x()) < epsilon) ? ((direction.x() >= 0) ? 1e20f : -1e20f) : (1.0f / direction.x());
            inverseDirection.y() = (std::abs(direction.y()) < epsilon) ? ((direction.y() >= 0) ? 1e20f : -1e20f) : (1.0f / direction.y());
            inverseDirection.z() = (std::abs(direction.z()) < epsilon) ? ((direction.z() >= 0) ? 1e20f : -1e20f) : (1.0f / direction.z());
        }

        inline bool IntersectSphere(const Eigen::Vector3f& sphereCenter, float radius, float& t) const
        {
            Eigen::Vector3f m = origin - sphereCenter;
            float b = m.dot(direction);
            float c = m.dot(m) - radius * radius;

            if (c > 0.0f && b > 0.0f) return false;

            float discr = b * b - c;

            if (discr < 0.0f) return false;

            t = -b - std::sqrt(discr);

            if (t < 0.0f) t = -b + std::sqrt(discr);

            return t >= 0.0f;
        }
    };

    struct AABB
    {
        Eigen::Vector3f min = Eigen::Vector3f::Constant(FLT_MAX);
        Eigen::Vector3f max = Eigen::Vector3f::Constant(-FLT_MAX);

        inline bool Intersects(const AABB& other) const
        {
            if (max.x() < other.min.x() || min.x() > other.max.x()) return false;
            if (max.y() < other.min.y() || min.y() > other.max.y()) return false;
            if (max.z() < other.min.z() || min.z() > other.max.z()) return false;

            return true;
        }

        inline bool Contains(const Eigen::Vector3f& p) const
        {
            return
                p.x() >= min.x() && p.x() <= max.x() &&
                p.y() >= min.y() && p.y() <= max.y() &&
                p.z() >= min.z() && p.z() <= max.z();
        }

        inline void Expand(const Eigen::Vector3f& p)
        {
            min = min.cwiseMin(p);
            max = max.cwiseMax(p);
        }

        inline void Expand(const AABB& other)
        {
            min = min.cwiseMin(other.min);
            max = max.cwiseMax(other.max);
        }

        inline bool IntersectRay(const Ray& ray, float& tNear, float& tFar) const
        {
            Eigen::Vector3f t0 = (min - ray.origin).cwiseProduct(ray.inverseDirection);
            Eigen::Vector3f t1 = (max - ray.origin).cwiseProduct(ray.inverseDirection);
            Eigen::Vector3f tMin = t0.cwiseMin(t1);
            Eigen::Vector3f tMax = t0.cwiseMax(t1);

            tNear = std::max(std::max(tMin.x(), tMin.y()), tMin.z());
            tFar = std::min(std::min(tMax.x(), tMax.y()), tMax.z());

            return tNear <= tFar && tFar >= 0.0f;
        }

    };

    class Configuration
    {
    public:
        static constexpr int voxelsPerBlockAxis = 8;
        static constexpr int voxelsPerBlock =
            voxelsPerBlockAxis * voxelsPerBlockAxis * voxelsPerBlockAxis;

        static constexpr float voxelSize = 0.3f;
        static constexpr int sdfOffset = 1;

        inline static Eigen::Vector3f filterMin = Eigen::Vector3f::Constant(-FLT_MAX);
        inline static Eigen::Vector3f filterMax = Eigen::Vector3f::Constant(FLT_MAX);

        static constexpr float pointVisualizationRadius = 0.025f;
    };

    class Triangle
    {
    public:
        Eigen::Vector3f v[3];
        Eigen::Vector3f n[3];
        Eigen::Vector3f c[3];
    };

    class PointCloud
    {
    public:
        size_t numberOfElements = 0;
        std::vector<Eigen::Vector3f> positions;
        std::vector<Eigen::Vector3f> normals;
        std::vector<Eigen::Vector3f> colors;
        std::vector<int> pointDeepLearningClassIDs;
        std::vector<int> pointClusterIDs;
        std::vector<int> marks;

        Eigen::AABB aabb;

        void Clear()
        {
            numberOfElements = 0;
            positions.clear();
            normals.clear();
            colors.clear();
            pointDeepLearningClassIDs.clear();
            pointClusterIDs.clear();
            marks.clear();
            aabb = Eigen::AABB();
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
			pc.Resize(numberOfElements);

			memcpy(pc.positions.data(), positions.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(pc.normals.data(), normals.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(pc.colors.data(), colors.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(pc.pointDeepLearningClassIDs.data(), pointDeepLearningClassIDs.data(), sizeof(int) * numberOfElements);
			memcpy(pc.pointClusterIDs.data(), pointClusterIDs.data(), sizeof(int) * numberOfElements);
			memcpy(pc.marks.data(), marks.data(), sizeof(int) * numberOfElements);
            pc.aabb = aabb;

            return pc;
        }

        void CopyFrom(const PointCloud& src)
        {
            Clear();
            Resize(src.numberOfElements);

			memcpy(positions.data(), src.positions.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(normals.data(), src.normals.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(colors.data(), src.colors.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(pointDeepLearningClassIDs.data(), src.pointDeepLearningClassIDs.data(), sizeof(int) * numberOfElements);
			memcpy(pointClusterIDs.data(), src.pointClusterIDs.data(), sizeof(int) * numberOfElements);
            memcpy(marks.data(), src.marks.data(), sizeof(int) * numberOfElements);
			aabb = src.aabb;
        }

        void CopyTo(PointCloud& dst) const
        {
            dst.Clear();
			dst.Resize(numberOfElements);

			memcpy(dst.positions.data(), positions.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(dst.normals.data(), normals.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(dst.colors.data(), colors.data(), sizeof(Eigen::Vector3f) * numberOfElements);
			memcpy(dst.pointDeepLearningClassIDs.data(), pointDeepLearningClassIDs.data(), sizeof(int) * numberOfElements);
			memcpy(dst.pointClusterIDs.data(), pointClusterIDs.data(), sizeof(int) * numberOfElements);
			memcpy(dst.marks.data(), marks.data(), sizeof(int) * numberOfElements);
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
                    colors[i] = Eigen::Vector3f(
                        ply.GetColors()[4 * i + 0],
                        ply.GetColors()[4 * i + 1],
                        ply.GetColors()[4 * i + 2]);
                }
            }
            if (ply.GetDeepLearningClasses().size() == numberOfElements)
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
                ply.AddPoint(positions[i].x(), positions[i].y(), positions[i].z());
                if (normals.size() == numberOfElements)
                {
                    ply.AddNormal(normals[i].x(), normals[i].y(), normals[i].z());
                }
                if (colors.size() == numberOfElements)
                {
                    ply.AddColor(colors[i].x(), colors[i].y(), colors[i].z());
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
        Eigen::Vector3f normal = Eigen::Vector3f::Zero();
        Eigen::Vector3f color = Eigen::Vector3f::Zero();
        int clusterId = -1;
        float divergence = 0.0f;
    };

    struct SparseGridPickResult
    {
        bool hasHit = false;
        int pointIndex = -1;
        uint64_t cellKey = 0;
        int gx = 0, gy = 0, gz = 0;
        float distance = 0.0f;
    };

    class ISpatialPartitioning {};

    class SparseGrid : public ISpatialPartitioning
    {
    public:
        robin_hood::unordered_flat_map<uint64_t, int> voxelPointListHead;
        std::vector<int> nextPoint;

        Eigen::AABB aabb;
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

            aabb.min -= Eigen::Vector3f::Constant(cellSize * 0.1f);
            aabb.max += Eigen::Vector3f::Constant(cellSize * 0.1f);

            for (int i = 0; i < (int)pc.numberOfElements; ++i)
            {
                int gx = (int)((pc.positions[i].x() - aabb.min.x()) / cellSize);
                int gy = (int)((pc.positions[i].y() - aabb.min.y()) / cellSize);
                int gz = (int)((pc.positions[i].z() - aabb.min.z()) / cellSize);

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

        int GetClosestPoint(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& queryPos, float& outDist)
        {
            outDist = FLT_MAX;
            if (points.empty()) return -1;

            int startGx = (int)std::floor((queryPos.x() - aabb.min.x()) / cellSize);
            int startGy = (int)std::floor((queryPos.y() - aabb.min.y()) / cellSize);
            int startGz = (int)std::floor((queryPos.z() - aabb.min.z()) / cellSize);

            float minDistSq = FLT_MAX;
            int closestIdx = -1;

            int searchRadius = 0;
            const int maxSearchRadius = 100;

            while (searchRadius < maxSearchRadius)
            {
                int minR = -searchRadius;
                int maxR = searchRadius;

                for (int dz = minR; dz <= maxR; ++dz)
                {
                    for (int dy = minR; dy <= maxR; ++dy)
                    {
                        for (int dx = minR; dx <= maxR; ++dx)
                        {
                            if (searchRadius > 0 && std::abs(dx) != searchRadius && std::abs(dy) != searchRadius && std::abs(dz) != searchRadius)
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
                                    Eigen::Vector3f diff = queryPos - points[currIdx];
                                    float sqDist = diff.squaredNorm();

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

                if (closestIdx != -1)
                {
                    float minX = aabb.min.x() + (startGx - searchRadius) * cellSize;
                    float maxX = aabb.min.x() + (startGx + searchRadius + 1) * cellSize;
                    float minY = aabb.min.y() + (startGy - searchRadius) * cellSize;
                    float maxY = aabb.min.y() + (startGy + searchRadius + 1) * cellSize;
                    float minZ = aabb.min.z() + (startGz - searchRadius) * cellSize;
                    float maxZ = aabb.min.z() + (startGz + searchRadius + 1) * cellSize;

                    float distToX = std::min(std::abs(queryPos.x() - minX), std::abs(queryPos.x() - maxX));
                    float distToY = std::min(std::abs(queryPos.y() - minY), std::abs(queryPos.y() - maxY));
                    float distToZ = std::min(std::abs(queryPos.z() - minZ), std::abs(queryPos.z() - maxZ));

                    float minDistToBoundary = std::min({ distToX, distToY, distToZ });

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

        void GetKNearestNeighbors(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& queryPos, int k, std::vector<unsigned int>& outIndices, std::vector<float>& outDistances)
        {
            outIndices.clear();
            outDistances.clear();
            if (points.empty() || k <= 0) return;

            std::priority_queue<std::pair<float, int>> pq;

            int startGx = (int)std::floor((queryPos.x() - aabb.min.x()) / cellSize);
            int startGy = (int)std::floor((queryPos.y() - aabb.min.y()) / cellSize);
            int startGz = (int)std::floor((queryPos.z() - aabb.min.z()) / cellSize);

            int searchRadius = 0;
            const int maxSearchRadius = 100;

            while (searchRadius < maxSearchRadius)
            {
                int minR = -searchRadius;
                int maxR = searchRadius;

                for (int dz = minR; dz <= maxR; ++dz)
                {
                    for (int dy = minR; dy <= maxR; ++dy)
                    {
                        for (int dx = minR; dx <= maxR; ++dx)
                        {
                            if (searchRadius > 0 && std::abs(dx) != searchRadius && std::abs(dy) != searchRadius && std::abs(dz) != searchRadius)
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
                                    Eigen::Vector3f diff = queryPos - points[currIdx];
                                    float sqDist = diff.squaredNorm();

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

                if (pq.size() == (size_t)k)
                {
                    float minX = aabb.min.x() + (startGx - searchRadius) * cellSize;
                    float maxX = aabb.min.x() + (startGx + searchRadius + 1) * cellSize;
                    float minY = aabb.min.y() + (startGy - searchRadius) * cellSize;
                    float maxY = aabb.min.y() + (startGy + searchRadius + 1) * cellSize;
                    float minZ = aabb.min.z() + (startGz - searchRadius) * cellSize;
                    float maxZ = aabb.min.z() + (startGz + searchRadius + 1) * cellSize;

                    float distToX = std::min(std::abs(queryPos.x() - minX), std::abs(queryPos.x() - maxX));
                    float distToY = std::min(std::abs(queryPos.y() - minY), std::abs(queryPos.y() - maxY));
                    float distToZ = std::min(std::abs(queryPos.z() - minZ), std::abs(queryPos.z() - maxZ));

                    float minDistToBoundary = std::min({ distToX, distToY, distToZ });

                    if (minDistToBoundary * minDistToBoundary > pq.top().first)
                    {
                        break;
                    }
                }

                searchRadius++;
            }

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

        SparseGridPickResult Pick(const std::vector<Eigen::Vector3f>& points, const Eigen::Ray& ray, float pickRadius)
        {
            SparseGridPickResult result;
            result.distance = FLT_MAX;
            result.hasHit = false;

            if (points.empty() || voxelPointListHead.empty()) return result;

            float tEntry = 0.0f, tExit = 0.0f;
            if (!aabb.IntersectRay(ray, tEntry, tExit)) return result;

            if (tEntry < 0.0f) tEntry = 0.0f;

            Eigen::Vector3f startPos = ray.origin + ray.direction * (tEntry + 0.001f);

            int curGx = (int)std::floor((startPos.x() - aabb.min.x()) / cellSize);
            int curGy = (int)std::floor((startPos.y() - aabb.min.y()) / cellSize);
            int curGz = (int)std::floor((startPos.z() - aabb.min.z()) / cellSize);

            if (curGx < 0) curGx = 0;
            if (curGy < 0) curGy = 0;
            if (curGz < 0) curGz = 0;

            int stepX = (ray.direction.x() >= 0) ? 1 : -1;
            int stepY = (ray.direction.y() >= 0) ? 1 : -1;
            int stepZ = (ray.direction.z() >= 0) ? 1 : -1;

            float nextBoundX = aabb.min.x() + (curGx + (stepX > 0 ? 1 : 0)) * cellSize;
            float nextBoundY = aabb.min.y() + (curGy + (stepY > 0 ? 1 : 0)) * cellSize;
            float nextBoundZ = aabb.min.z() + (curGz + (stepZ > 0 ? 1 : 0)) * cellSize;

            float tMaxX = (nextBoundX - ray.origin.x()) * ray.inverseDirection.x();
            float tMaxY = (nextBoundY - ray.origin.y()) * ray.inverseDirection.y();
            float tMaxZ = (nextBoundZ - ray.origin.z()) * ray.inverseDirection.z();

            float tDeltaX = std::abs(cellSize * ray.inverseDirection.x());
            float tDeltaY = std::abs(cellSize * ray.inverseDirection.y());
            float tDeltaZ = std::abs(cellSize * ray.inverseDirection.z());

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

                    float tNextBoundary = std::min(std::min(tMaxX, tMaxY), tMaxZ);
                    if (result.hasHit && result.distance <= tNextBoundary)
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

        SparseGridPickResult PickBruteForce(const std::vector<Eigen::Vector3f>& points, const Ray& ray, float pickRadius)
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

                Eigen::Vector3f cellMin = aabb.min + Eigen::Vector3f((float)gx * cellSize, (float)gy * cellSize, (float)gz * cellSize);
                Eigen::Vector3f cellMax = cellMin + Eigen::Vector3f::Constant(cellSize);

                VD::AddWiredBox("SparseGridCells", { cellMin, cellMax }, Eigen::Vector4f(0.0f, 1.0f, 0.0f, 0.3f));

                int currIdx = headIdx;
                while (currIdx != -1)
                {
                    const Eigen::Vector3f& p = pc.positions[currIdx];
                    const Eigen::Vector3f& n = pc.normals[currIdx].normalized();
                    const Eigen::Vector3f& c = pc.colors[currIdx];

                    VD::AddSphere("SparseGridPoints", p, n, GeometricProcessingPipeline::Configuration::pointVisualizationRadius, Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f));

                    currIdx = nextPoint[currIdx];
                }
            }
        }
    };

    typedef uint64_t DataBlockKey;

    class DataBlock
    {
    public:
        Eigen::Vector3f blockMin = Eigen::Vector3f::Zero();
        Voxel voxels[Configuration::voxelsPerBlock];
        std::mutex blockMutex;

        void Initialize()
        {
            for (int i = 0; i < Configuration::voxelsPerBlock; ++i)
            {
                voxels[i] = Voxel();
            }
        }
    };

    struct SparseDataBlock
    {
        float voxelSize = Configuration::voxelSize;
        Eigen::Vector3f gridOrigin = Eigen::Vector3f::Zero();
        std::unordered_map<DataBlockKey, std::unique_ptr<DataBlock>> dataBlocks;

        float blockSizePerAxis = voxelSize * Configuration::voxelsPerBlockAxis;

        Voxel* GetVoxelByIndex(int gx, int gy, int gz)
        {
            int bx = (int)floor((float)gx / Configuration::voxelsPerBlockAxis);
            int by = (int)floor((float)gy / Configuration::voxelsPerBlockAxis);
            int bz = (int)floor((float)gz / Configuration::voxelsPerBlockAxis);

            Eigen::Vector3f blockMin = gridOrigin + Eigen::Vector3f((float)bx * blockSizePerAxis, (float)by * blockSizePerAxis, (float)bz * blockSizePerAxis);
            auto key = Morton3D::EncodeFromVec3(blockMin + Eigen::Vector3f::Constant(voxelSize * 0.1f), gridOrigin, blockSizePerAxis);

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

        void FromPointsData(const std::vector<Eigen::Vector3f>& points,
            const std::vector<Eigen::Vector3f>& normals,
            const std::vector<Eigen::Vector3f>& colors,
            const std::vector<int>& clusterIds,
            const Eigen::Vector3f& aabbMin)
        {
            blockSizePerAxis = voxelSize * Configuration::voxelsPerBlockAxis;

            gridOrigin.x() = std::floor(aabbMin.x() / blockSizePerAxis) * blockSizePerAxis;
            gridOrigin.y() = std::floor(aabbMin.y() / blockSizePerAxis) * blockSizePerAxis;
            gridOrigin.z() = std::floor(aabbMin.z() / blockSizePerAxis) * blockSizePerAxis;

            TS(Occupy);

            float truncDist = std::max(voxelSize * 4.0f, 0.15f);

            size_t numPoints = points.size();
            std::vector<size_t> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            {
                TS(Pass1_Alloc);

                using KeyPair = std::pair<DataBlockKey, Eigen::Vector3f>;

                std::vector<KeyPair> keysToAllocate;
                keysToAllocate.reserve(numPoints * 2);
                std::mutex vecMutex;

                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
                    {
                        std::vector<KeyPair> localKeys;
                        localKeys.reserve(8);

                        Eigen::Vector3f p = points[i];
                        Eigen::Vector3f vecFromOrigin = p - gridOrigin;

                        int centerGx = (int)std::floor(vecFromOrigin.x() / voxelSize);
                        int centerGy = (int)std::floor(vecFromOrigin.y() / voxelSize);
                        int centerGz = (int)std::floor(vecFromOrigin.z() / voxelSize);

                        int bx = centerGx / Configuration::voxelsPerBlockAxis;
                        int by = centerGy / Configuration::voxelsPerBlockAxis;
                        int bz = centerGz / Configuration::voxelsPerBlockAxis;

                        Eigen::Vector3f blockMin = gridOrigin + Eigen::Vector3f((float)bx * blockSizePerAxis, (float)by * blockSizePerAxis, (float)bz * blockSizePerAxis);
                        auto key = Morton3D::EncodeFromVec3(blockMin + Eigen::Vector3f::Constant(voxelSize * 0.1f), gridOrigin, blockSizePerAxis);
                        localKeys.push_back({ key, blockMin });

                        Eigen::Vector3f localP = p - blockMin;

                        float margin = truncDist + voxelSize * 0.5f;

                        bool nearX_Neg = localP.x() < margin;
                        bool nearX_Pos = localP.x() > blockSizePerAxis - margin;
                        bool nearY_Neg = localP.y() < margin;
                        bool nearY_Pos = localP.y() > blockSizePerAxis - margin;
                        bool nearZ_Neg = localP.z() < margin;
                        bool nearZ_Pos = localP.z() > blockSizePerAxis - margin;

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

                                        Eigen::Vector3f nbMin = gridOrigin + Eigen::Vector3f((float)(bx + dx) * blockSizePerAxis, (float)(by + dy) * blockSizePerAxis, (float)(bz + dz) * blockSizePerAxis);
                                        auto nKey = Morton3D::EncodeFromVec3(nbMin + Eigen::Vector3f::Constant(voxelSize * 0.1f), gridOrigin, blockSizePerAxis);
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

            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
                {
                    auto p = points[i];
                    Eigen::Vector3f n = (normals.empty()) ? Eigen::Vector3f(0, 1, 0) : normals[i];
                    Eigen::Vector3f c = (colors.empty()) ? Eigen::Vector3f(1, 1, 1) : colors[i];
                    int cid = (clusterIds.empty()) ? -1 : clusterIds[i];

                    if (c.x() > 1.0f) c /= 255.0f;

                    Eigen::Vector3f vecFromOrigin = p - gridOrigin;
                    int centerGx = (int)std::floor(vecFromOrigin.x() / voxelSize);
                    int centerGy = (int)std::floor(vecFromOrigin.y() / voxelSize);
                    int centerGz = (int)std::floor(vecFromOrigin.z() / voxelSize);

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

                                Eigen::Vector3f voxelCenter = gridOrigin + Eigen::Vector3f((gx + 0.5f) * voxelSize, (gy + 0.5f) * voxelSize, (gz + 0.5f) * voxelSize);
                                float dist = (p - voxelCenter).norm();
                                if (dist > truncDist) continue;

                                int bx = (int)floor((float)gx / Configuration::voxelsPerBlockAxis);
                                int by = (int)floor((float)gy / Configuration::voxelsPerBlockAxis);
                                int bz = (int)floor((float)gz / Configuration::voxelsPerBlockAxis);

                                float currBlockSize = blockSizePerAxis;
                                Eigen::Vector3f blockMin = gridOrigin + Eigen::Vector3f(bx * currBlockSize, by * currBlockSize, bz * currBlockSize);
                                auto key = Morton3D::EncodeFromVec3(blockMin + Eigen::Vector3f::Constant(voxelSize * 0.1f), gridOrigin, currBlockSize);

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
                                    float sdf = std::clamp((voxelCenter - p).dot(n), -truncDist, truncDist);

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
                                        voxel.divergence = 0.0f;
                                    }
                                    else
                                    {
                                        float newW = voxel.weight + weight;

                                        Eigen::Vector3f currentDir = voxel.normal.normalized();
                                        float dotVal = std::clamp(currentDir.dot(n), -1.0f, 1.0f);
                                        float newDiv = 1.0f - dotVal;

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
                Eigen::Vector3f blockMax = block->blockMin + Eigen::Vector3f::Constant(blockSizePerAxis);
                VD::AddWiredBox("Blocks", { glm::vec3(block->blockMin.x(), block->blockMin.y(), block->blockMin.z()), glm::vec3(blockMax.x(), blockMax.y(), blockMax.z()) }, Color::yellow());

                for (int z = 0; z < Configuration::voxelsPerBlockAxis; ++z)
                {
                    for (int y = 0; y < Configuration::voxelsPerBlockAxis; ++y)
                    {
                        for (int x = 0; x < Configuration::voxelsPerBlockAxis; ++x)
                        {
                            const Voxel& voxel = block->voxels[z * Configuration::voxelsPerBlockAxis * Configuration::voxelsPerBlockAxis + y * Configuration::voxelsPerBlockAxis + x];
                            if (voxel.valid)
                            {
                                Eigen::Vector3f vMin = block->blockMin + Eigen::Vector3f((float)x * voxelSize, (float)y * voxelSize, (float)z * voxelSize);
                                Eigen::Vector3f vMax = vMin + Eigen::Vector3f::Constant(voxelSize);
                                VD::AddWiredBox("Voxels", { glm::vec3(vMin.x(), vMin.y(), vMin.z()), glm::vec3(vMax.x(), vMax.y(), vMax.z()) }, Color::red());
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
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> holeEdges;

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
            Eigen::Vector3f pos;
            Eigen::Vector3f normal;
            Eigen::Vector3f color;
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

            const Eigen::Vector3i corners[8] = { {0,0,0}, {1,0,0}, {1,0,1}, {0,0,1}, {0,1,0}, {1,1,0}, {1,1,1}, {0,1,1} };
            const int edgePairs[12][2] = { {0,1}, {1,2}, {2,3}, {3,0}, {4,5}, {5,6}, {6,7}, {7,4}, {0,4}, {1,5}, {2,6}, {3,7} };

            std::for_each(std::execution::par, blocks.begin(), blocks.end(), [&](DataBlock* block)
                {
                    std::vector<std::pair<GridKey, SNVertex>> localVerts;
                    localVerts.reserve(64);

                    Eigen::Vector3f diff = block->blockMin - sdb.gridOrigin;
                    int startGx = (int)(diff.x() / sdb.voxelSize + 0.5f);
                    int startGy = (int)(diff.y() / sdb.voxelSize + 0.5f);
                    int startGz = (int)(diff.z() / sdb.voxelSize + 0.5f);

                    for (int z = 0; z < Configuration::voxelsPerBlockAxis; ++z)
                    {
                        for (int y = 0; y < Configuration::voxelsPerBlockAxis; ++y)
                        {
                            for (int x = 0; x < Configuration::voxelsPerBlockAxis; ++x)
                            {
                                int gx = startGx + x; int gy = startGy + y; int gz = startGz + z;

                                float dists[8]; Eigen::Vector3f colors[8], normals[8];
                                int insideCount = 0; bool allValid = true;

                                for (int i = 0; i < 8; ++i)
                                {
                                    const auto* v = sdb.GetVoxelByIndex(gx + corners[i].x(), gy + corners[i].y(), gz + corners[i].z());
                                    if (!v || !v->valid)
                                    {
                                        allValid = false;
                                        break;
                                    }
                                    dists[i] = v->signedDistance; colors[i] = v->color; normals[i] = v->normal;
                                    if (dists[i] < isoLevel) insideCount++;
                                }

                                if (!allValid || insideCount == 0 || insideCount == 8) continue;

                                Eigen::Vector3f avgPos(0.0f, 0.0f, 0.0f), avgColor(0.0f, 0.0f, 0.0f), avgNormal(0.0f, 0.0f, 0.0f);
                                int intersections = 0;
                                for (int e = 0; e < 12; ++e)
                                {
                                    int idx1 = edgePairs[e][0]; int idx2 = edgePairs[e][1];
                                    if ((dists[idx1] < isoLevel) != (dists[idx2] < isoLevel))
                                    {
                                        float t = (isoLevel - dists[idx1]) / (dists[idx2] - dists[idx1]);
                                        Eigen::Vector3f p1 = sdb.gridOrigin + Eigen::Vector3f(gx + corners[idx1].x(), gy + corners[idx1].y(), gz + corners[idx1].z()) * sdb.voxelSize;
                                        Eigen::Vector3f p2 = sdb.gridOrigin + Eigen::Vector3f(gx + corners[idx2].x(), gy + corners[idx2].y(), gz + corners[idx2].z()) * sdb.voxelSize;

                                        avgPos += (p1 * (1.0f - t) + p2 * t);
                                        avgColor += (colors[idx1] * (1.0f - t) + colors[idx2] * t);
                                        avgNormal += (normals[idx1] * (1.0f - t) + normals[idx2] * t);
                                        intersections++;
                                    }
                                }

                                if (intersections > 0)
                                {
                                    SNVertex v;
                                    v.pos = avgPos / (float)intersections;
                                    v.color = avgColor / (float)intersections;
                                    v.normal = avgNormal.normalized();
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

                    Eigen::Vector3f diff = block->blockMin - sdb.gridOrigin;
                    int startGx = (int)(diff.x() / sdb.voxelSize + 0.5f);
                    int startGy = (int)(diff.y() / sdb.voxelSize + 0.5f);
                    int startGz = (int)(diff.z() / sdb.voxelSize + 0.5f);

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
                std::vector<Eigen::Vector3f> v, n, c; std::vector<uint32_t> ind;
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
                for (const auto& edge : holeEdges)
                {
                    VD::AddLine("Holes", glm::vec3(edge.first.x(), edge.first.y(), edge.first.z()), glm::vec3(edge.second.x(), edge.second.y(), edge.second.z()), Color::red());
                }
            }
        }

        void ExportPLY(const std::string& filename)
        {
            PLYFormat ply;
            for (const auto& t : triangles)
            {
                ply.AddPoint(t.v[0].x(), t.v[0].y(), t.v[0].z()); ply.AddNormal(t.n[0].x(), t.n[0].y(), t.n[0].z()); ply.AddColor(t.c[0].x(), t.c[0].y(), t.c[0].z());
                ply.AddPoint(t.v[1].x(), t.v[1].y(), t.v[1].z()); ply.AddNormal(t.n[1].x(), t.n[1].y(), t.n[1].z()); ply.AddColor(t.c[1].x(), t.c[1].y(), t.c[1].z());
                ply.AddPoint(t.v[2].x(), t.v[2].y(), t.v[2].z()); ply.AddNormal(t.n[2].x(), t.n[2].y(), t.n[2].z()); ply.AddColor(t.c[2].x(), t.c[2].y(), t.c[2].z());
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
            std::vector<Eigen::Vector3f> tempVerts; tempVerts.reserve(triangles.size());

            for (const auto& t : triangles)
            {
                for (int i = 0; i < 3; ++i)
                {
                    GridKey key = { (int)(t.v[i].x() / tol), (int)(t.v[i].y() / tol), (int)(t.v[i].z() / tol) };
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
            : pipeline(pipeline), parameter(parameter) {
        }
        virtual ~IGeometricProcessingOperator()
        {
            if (parameter.needToDeleteSpatialPartitioning)
            {
                SAFE_DELETE(spatialPartitioning);
            }
        }

        virtual void Process() = 0;
        void Process(SpatialPartitioningType* spatialPartitioning)
        {
            this->spatialPartitioning = spatialPartitioning;
            Process();
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
        std::shared_ptr<PointCloud> cachedPointCloud = nullptr;
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

        int CreatePointCloud();
        void StorePointCloud();
        void RestoreInitialPointCloud();
        void RestoreLastPointCloud();

        inline SparseGrid* GetSparseGrid() const { return sparseGrid; }

        inline std::shared_ptr<PointCloud> GetCurrentPointCloud()
        {
            if(-1 == currentPointCloudIndex || currentPointCloudIndex >= pointClouds.size()) return nullptr;
            else return pointClouds[currentPointCloudIndex];
        }

		inline std::shared_ptr<PointCloud> GetInitialPointCloud() { return pointClouds.empty() ? nullptr : pointClouds[0]; }
        inline std::shared_ptr<PointCloud> GetLastPointCloud() { return pointClouds.empty() ? nullptr : pointClouds.back(); }
        inline std::shared_ptr<PointCloud> GetPointCloud(int index)
        {
            if (-1 == index) return GetLastPointCloud();
            else if(pointClouds.empty()) return nullptr;
            else return pointClouds[index];
        }

    protected:
        std::vector<std::tuple<std::string, std::shared_ptr<IGeometricProcessingOperator<SparseGrid>>>> operators;
        SparseGrid* sparseGrid = nullptr;
        std::vector<Triangle> generatedMeshTriangles;

        std::vector<std::shared_ptr<PointCloud>> pointClouds;
        int currentPointCloudIndex = -1;
    };
}
