#include "GeometricProcessingPipeline.h"

#include <FeatherCommon.h>

using VD = VisualDebugging;

namespace GeometricProcessingPipeline
{
#pragma region Ray
	Ray::Ray(const Eigen::Vector3f& o, const Eigen::Vector3f& d) : origin(o), direction(d) {
		const float epsilon = 1e-6f;

		inverseDirection.x() = (std::abs(direction.x()) < epsilon) ? ((direction.x() >= 0) ? 1e20f : -1e20f) : (1.0f / direction.x());
		inverseDirection.y() = (std::abs(direction.y()) < epsilon) ? ((direction.y() >= 0) ? 1e20f : -1e20f) : (1.0f / direction.y());
		inverseDirection.z() = (std::abs(direction.z()) < epsilon) ? ((direction.z() >= 0) ? 1e20f : -1e20f) : (1.0f / direction.z());
	}

	bool Ray::IntersectSphere(const Eigen::Vector3f& sphereCenter, float radius, float& t) const
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
#pragma endregion

#pragma region AABB
	bool AABB::Intersects(const AABB& other) const
	{
		if (max.x() < other.min.x() || min.x() > other.max.x()) return false;
		if (max.y() < other.min.y() || min.y() > other.max.y()) return false;
		if (max.z() < other.min.z() || min.z() > other.max.z()) return false;

		return true;
	}

	bool AABB::Contains(const Eigen::Vector3f& p) const
	{
		return
			p.x() >= min.x() && p.x() <= max.x() &&
			p.y() >= min.y() && p.y() <= max.y() &&
			p.z() >= min.z() && p.z() <= max.z();
	}

	void AABB::Expand(const Eigen::Vector3f& p)
	{
		min = min.cwiseMin(p);
		max = max.cwiseMax(p);
	}

	void AABB::Expand(const AABB& other)
	{
		min = min.cwiseMin(other.min);
		max = max.cwiseMax(other.max);
	}

	bool AABB::IntersectRay(const Ray& ray, float& tNear, float& tFar) const
	{
		Eigen::Vector3f t0 = (min - ray.origin).cwiseProduct(ray.inverseDirection);
		Eigen::Vector3f t1 = (max - ray.origin).cwiseProduct(ray.inverseDirection);
		Eigen::Vector3f tMin = t0.cwiseMin(t1);
		Eigen::Vector3f tMax = t0.cwiseMax(t1);

		tNear = std::max(std::max(tMin.x(), tMin.y()), tMin.z());
		tFar = std::min(std::min(tMax.x(), tMax.y()), tMax.z());

		return tNear <= tFar && tFar >= 0.0f;
	}
#pragma endregion

#pragma region PointCloud
	void PointCloud::Clear()
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

	void PointCloud::Resize(size_t newSize)
	{
		numberOfElements = newSize;
		positions.resize(newSize);
		normals.resize(newSize);
		colors.resize(newSize);
		pointDeepLearningClassIDs.resize(newSize, -1);
		pointClusterIDs.resize(newSize, -1);
		for (auto& [k, v] : marks)
		{
			v.resize(newSize, -1);
		}
	}

	[[nodiscard]] PointCloud PointCloud::Clone() const
	{
		PointCloud pc;
		pc.Resize(numberOfElements);

		memcpy(pc.positions.data(), positions.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(pc.normals.data(), normals.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(pc.colors.data(), colors.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(pc.pointDeepLearningClassIDs.data(), pointDeepLearningClassIDs.data(), sizeof(int) * numberOfElements);
		memcpy(pc.pointClusterIDs.data(), pointClusterIDs.data(), sizeof(int) * numberOfElements);

		for (auto& [key, value] : marks)
		{
			pc.marks[key].resize(numberOfElements);
			memcpy(pc.marks[key].data(), value.data(), sizeof(int) * numberOfElements);
		}

		pc.aabb = aabb;

		return pc;
	}

	void PointCloud::CopyFrom(const PointCloud& src)
	{
		Clear();
		Resize(src.numberOfElements);

		memcpy(positions.data(), src.positions.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(normals.data(), src.normals.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(colors.data(), src.colors.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(pointDeepLearningClassIDs.data(), src.pointDeepLearningClassIDs.data(), sizeof(int) * numberOfElements);
		memcpy(pointClusterIDs.data(), src.pointClusterIDs.data(), sizeof(int) * numberOfElements);

		for (const auto& [key, value] : src.marks)
		{
			marks[key].resize(numberOfElements);
			memcpy(marks[key].data(), value.data(), sizeof(int) * numberOfElements);
		}

		aabb = src.aabb;
	}

	void PointCloud::CopyTo(PointCloud& dst) const
	{
		dst.Clear();
		dst.Resize(numberOfElements);

		memcpy(dst.positions.data(), positions.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(dst.normals.data(), normals.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(dst.colors.data(), colors.data(), sizeof(Eigen::Vector3f) * numberOfElements);
		memcpy(dst.pointDeepLearningClassIDs.data(), pointDeepLearningClassIDs.data(), sizeof(int) * numberOfElements);
		memcpy(dst.pointClusterIDs.data(), pointClusterIDs.data(), sizeof(int) * numberOfElements);

		for (const auto& [key, value] : marks)
		{
			dst.marks[key].resize(numberOfElements);
			memcpy(dst.marks[key].data(), value.data(), sizeof(int) * numberOfElements);
		}

		dst.aabb = aabb;
	}

	void PointCloud::FromPLY(const std::string& plyFileName)
	{
		PLYFormat ply;
		ply.Deserialize(plyFileName);
		FromPLY(ply);
	}

	void PointCloud::FromPLY(const PLYFormat& ply)
	{
		Clear();

		Resize(ply.GetPoints().size() / 3);

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

	void PointCloud::ToPLY(const std::string& plyFileName) const
	{
		PLYFormat ply;
		ToPLY(ply);
		ply.Serialize(plyFileName);
	}

	void PointCloud::ToPLY(PLYFormat& ply) const
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
#pragma endregion

#pragma region SparseGrid
	uint64_t SparseGrid::GetKey(int x, int y, int z) const
	{
		return ((uint64_t)x << 42) | ((uint64_t)y << 21) | (uint64_t)z;
	}

	void SparseGrid::Build(const GeometricProcessingPipeline::PointCloud& pc, float cellSize)
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

	int SparseGrid::GetClosestPoint(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& queryPos, float& outDist)
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

	void SparseGrid::GetKNearestNeighbors(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& queryPos, int k, std::vector<unsigned int>& outIndices, std::vector<float>& outDistances)
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

	SparseGridPickResult SparseGrid::Pick(const std::vector<Eigen::Vector3f>& points, const Eigen::Ray& ray, float pickRadius)
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

	SparseGridPickResult SparseGrid::PickBruteForce(const std::vector<Eigen::Vector3f>& points, const Eigen::Ray& ray, float pickRadius)
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

	void SparseGrid::Visualize(const GeometricProcessingPipeline::PointCloud& pc)
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
#pragma endregion

#pragma region SparseDataBlock
	void DataBlock::Initialize()
	{
		for (int i = 0; i < Configuration::voxelsPerBlock; ++i)
		{
			voxels[i] = Voxel();
		}
	}

	Voxel* SparseDataBlock::GetVoxelByIndex(int gx, int gy, int gz)
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

	void SparseDataBlock::FromPointsData(const std::vector<Eigen::Vector3f>& points,
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

		size_t numberOfPoints = points.size();
		std::vector<size_t> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		{
			TS(Pass1_Alloc);

			using KeyPair = std::pair<DataBlockKey, Eigen::Vector3f>;

			std::vector<KeyPair> keysToAllocate;
			keysToAllocate.reserve(numberOfPoints * 2);
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

	void SparseDataBlock::Visualize()
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
#pragma endregion

#pragma region MeshGenerator
	void MeshGenerator::Generate(SparseDataBlock& sdb)
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

	void MeshGenerator::Visualize(bool showMesh, bool showHoles)
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

	void MeshGenerator::ExportPLY(const std::string& filename)
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

	void MeshGenerator::DetectHoles()
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
#pragma endregion

#pragma region Pipeline
	Pipeline::~Pipeline()
	{
		Clear();
	}

	void Pipeline::BuildSparseGrid(PointCloud& pointCloud)
	{
		SAFE_DELETE(sparseGrid);

		sparseGrid = new SparseGrid();
		sparseGrid->Build(pointCloud, Configuration::voxelSize);
	}

	void Pipeline::Execute()
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

				op->Process(sparseGrid);

				auto operatorMeshGeneration = std::dynamic_pointer_cast<OperatorMeshGeneration>(op);
				if (operatorMeshGeneration)
				{
					generatedMeshTriangles = operatorMeshGeneration->GetTriangles();
				}

				std::cout << Miliseconds(time, tag.c_str()) << std::endl;
			}

			if (op->GetParameter().needToRebuildSpatialPartitioning)
			{
				auto time = std::chrono::high_resolution_clock::now();
				auto currentPointCloud = GetCurrentPointCloud();
				BuildSparseGrid(*currentPointCloud);
				std::cout << Miliseconds(time, "Rebuilding Spatial Grid") << std::endl;
			}

			printf("\n");
		}

		TE(GeometricProcessingPipeline);
	}

	void Pipeline::VisualizeAll()
	{
		for (auto& [tag, op] : operators)
		{
			op->Visualize();
		}
	}

	void Pipeline::VisualizeLast()
	{
		if (!operators.empty())
		{
			auto& [tag, op] = operators.back();
			op->Visualize();
		}
	}

	void Pipeline::Clear()
	{
		operators.clear();

		SAFE_DELETE(sparseGrid);
	}

	int Pipeline::CreatePointCloud()
	{
		pointClouds.clear();

		auto pc = std::make_shared<PointCloud>();
		pointClouds.push_back(pc);

		currentPointCloudIndex = 0;
		return currentPointCloudIndex;
	}

	void Pipeline::StorePointCloud()
	{
		if (currentPointCloudIndex < 0 || currentPointCloudIndex >= (int)pointClouds.size())
			return;

		auto newPC = std::make_shared<PointCloud>();
		newPC->CopyFrom(*pointClouds[currentPointCloudIndex]);

		pointClouds.push_back(newPC);
		currentPointCloudIndex = static_cast<int>(pointClouds.size()) - 1;
	}

	void Pipeline::RestoreInitialPointCloud()
	{
		if (pointClouds.empty())
			return;

		currentPointCloudIndex = 0;
	}

	void Pipeline::RestoreLastPointCloud()
	{
		if (pointClouds.empty())
			return;

		currentPointCloudIndex = static_cast<int>(pointClouds.size()) - 1;
	}
#pragma endregion

#pragma region OperatorPointCloudLoader
	OperatorPointCloudLoader::OperatorPointCloudLoader(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorPointCloudLoader::Process()
	{
		plyFilename = parameter.GetParameter<std::string>("plyFilename", "");

		if (plyFilename.empty()) return;

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (nullptr == currentPointCloud)
		{
			pipeline->CreatePointCloud();
			currentPointCloud = pipeline->GetCurrentPointCloud();
		}
		currentPointCloud->FromPLY(plyFilename);

		TS(PointCloudLoader);
		if (currentPointCloud->numberOfElements == 0) return;
		if (nullptr == spatialPartitioning)
		{
			parameter.needToRebuildSpatialPartitioning = true;
		}
		cachedPointCloud = currentPointCloud;
		TE(PointCloudLoader);
	}

	void OperatorPointCloudLoader::Visualize()
	{
		for (size_t i = 0; i < cachedPointCloud->numberOfElements; i++)
		{
			const auto& p = cachedPointCloud->positions[i];
			const auto& n = cachedPointCloud->normals[i];
			const auto& c = cachedPointCloud->colors[i];

			VD::AddSphere("PointCloudLoader",
				p,
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f)
			);
		}
	}
#pragma endregion

#pragma region OperatorPointCloudSaver
	OperatorPointCloudSaver::OperatorPointCloudSaver(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorPointCloudSaver::Process()
	{
		plyFilename = parameter.GetParameter<std::string>("plyFilename", "");

		if (plyFilename.empty()) return;

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (nullptr == currentPointCloud)
		{
			aerr("PointCloudSaver: No current point cloud to save.\n");
			return;
		}

		cachedPointCloud = currentPointCloud;
		
		currentPointCloud->ToPLY(plyFilename);
	}

	void OperatorPointCloudSaver::Visualize()
	{
		for (size_t i = 0; i < cachedPointCloud->numberOfElements; i++)
		{
			const auto& p = cachedPointCloud->positions[i];
			const auto& n = cachedPointCloud->normals[i];
			const auto& c = cachedPointCloud->colors[i];

			VD::AddSphere("PointCloudSaver",
				p,
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f)
			);
		}
	}
#pragma endregion

#pragma region OperatorStorePointCloud
	OperatorStorePointCloud::OperatorStorePointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorStorePointCloud::Process()
	{
		TS(StorePointCloud);
		pipeline->StorePointCloud();
		TE(StorePointCloud);
	}

	void OperatorStorePointCloud::Visualize()
	{
	}
#pragma endregion

#pragma region OperatorRestoreInitialPointCloud
	OperatorRestoreInitialPointCloud::OperatorRestoreInitialPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorRestoreInitialPointCloud::Process()
	{
		TS(StorePointCloud);
		pipeline->RestoreInitialPointCloud();
		TE(StorePointCloud);
	}

	void OperatorRestoreInitialPointCloud::Visualize()
	{
	}
#pragma endregion

#pragma region OperatorRestoreLastPointCloud
	OperatorRestoreLastPointCloud::OperatorRestoreLastPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorRestoreLastPointCloud::Process()
	{
		TS(StorePointCloud);
		pipeline->RestoreLastPointCloud();
		TE(StorePointCloud);
	}

	void OperatorRestoreLastPointCloud::Visualize()
	{
	}
#pragma endregion

#pragma region OperatorPointCloudVisualization
	OperatorPointCloudVisualization::OperatorPointCloudVisualization(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorPointCloudVisualization::Process()
	{
		TS(PointCloudVisualization);
		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;
		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}
		cachedPointCloud = currentPointCloud;
		TE(PointCloudVisualization);
	}

	void OperatorPointCloudVisualization::Visualize()
	{
		if (nullptr == cachedPointCloud) return;
		size_t count = cachedPointCloud->numberOfElements;
		for (size_t i = 0; i < count; ++i)
		{
			VD::AddSphere(
				"PointCloudVisualization",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius * 0.9f,
				Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f)
			);
		}
	}
#pragma endregion

#pragma region OperatorPointCloudDensity
	OperatorPointCloudDensity::OperatorPointCloudDensity(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorPointCloudDensity::Process()
	{
		TS(PointCloudDensity);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		// 파라미터 로드
		{
			searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
			visualizationScale = parameter.GetParameter<float>("visualizationScale", visualizationScale);
			// [수정] 탐색 오프셋 파라미터 로드
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
		}

		pointDensities.assign(numberOfPoints, 0.0f);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		// [Step 1] 병렬 처리: 밀도 계산
		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];

				int neighborCount = 0;

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

				// [수정] neighborSearchOffset을 사용하여 탐색 범위 확장
				for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz) {
					for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy) {
						for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx) {

							uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
							auto it = spatialPartitioning->voxelPointListHead.find(key);
							if (it == spatialPartitioning->voxelPointListHead.end()) continue;

							int curr = it->second;
							while (curr != -1) {
								if (curr != i) {
									float distSq = (p - currentPointCloud->positions[curr]).squaredNorm();
									if (distSq <= searchRadiusSq) {
										neighborCount++;
									}
								}
								curr = spatialPartitioning->nextPoint[curr];
							}
						}
					}
				}

				pointDensities[i] = (float)neighborCount;
			});

		// [Step 2] 통계 계산
		double sum = 0.0;
		double sqSum = 0.0;

		for (float d : pointDensities)
		{
			sum += d;
			sqSum += d * d;
		}

		densityMean = (float)(sum / numberOfPoints);
		double variance = (sqSum / numberOfPoints) - (densityMean * densityMean);
		densityStdDev = std::sqrt(std::max(0.0, variance));

		printf("Density Analysis: Mean=%.2f, StdDev=%.2f (Neighbors within R=%.3f, Offset=%d)\n",
			densityMean, densityStdDev, searchRadius, neighborSearchOffset);

		TE(PointCloudDensity);
	}

	void OperatorPointCloudDensity::Visualize()
	{
		// Visualize 함수는 이전과 동일하게 유지하거나, 필요 시 수정
		if (nullptr == cachedPointCloud || pointDensities.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		float rangeHalf = densityStdDev * visualizationScale;
		float minVal = std::max(0.0f, densityMean - rangeHalf);
		float maxVal = densityMean + rangeHalf;
		float range = maxVal - minVal;
		if (range < 1e-6f) range = 1.0f;

		/*
		for (size_t i = 0; i < count; ++i)
		{
			float val = pointDensities[i];
			float t = std::clamp((val - minVal) / range, 0.0f, 1.0f);

			Eigen::Vector3f color;
			// Blue(Low) -> Green -> Red(High)
			if (t < 0.5f)
			{
				float localT = t / 0.5f;
				color = Eigen::Vector3f(0.0f, localT, 1.0f - localT);
			}
			else
			{
				float localT = (t - 0.5f) / 0.5f;
				color = Eigen::Vector3f(localT, 1.0f - localT, 0.0f);
			}

			VD::AddSphere(
				"DensityHeatmap",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius, // 크기는 일정하게
				Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
			);
		}
		*/
		for (size_t i = 0; i < count; ++i)
		{
			float val = pointDensities[i];
			float t = std::clamp((val - minVal) / range, 0.0f, 1.0f);

			Eigen::Vector3f color;
			if (t > 0.75f)
			{
				VD::AddSphere(
					"DensityHeatmap",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius, // 크기는 일정하게
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f)
				);
			}
			else
			{
				auto& color = cachedPointCloud->colors[i];
				VD::AddSphere(
					"DensityHeatmap",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius, // 크기는 일정하게
					Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
				);
			}
		}
	}
#pragma endregion

#pragma region OperatorMeanShift
	OperatorMeanShift::OperatorMeanShift(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorMeanShift::Process()
	{
		TS(MeanShift);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		// 파라미터 로드
		{
			bandwidthMultiplier = parameter.GetParameter<float>("bandwidthMultiplier", bandwidthMultiplier);
			convergenceThreshold = parameter.GetParameter<float>("convergenceThreshold", convergenceThreshold);
			maxIterations = parameter.GetParameter<int>("maxIterations", maxIterations);
			updatePositions = parameter.GetParameter<bool>("updatePositions", updatePositions);
			visualizationScale = parameter.GetParameter<float>("visualizationScale", visualizationScale);

			invertDirection = parameter.GetParameter<bool>("invertDirection", invertDirection);
			markedPointsOnly = parameter.GetParameter<bool>("markedPointsOnly", markedPointsOnly); // [로드]
		}

		shiftedPositions.resize(numberOfPoints);
		shiftDistances.assign(numberOfPoints, 0.0f);

		float bandwidth = spatialPartitioning->cellSize * bandwidthMultiplier;
		float bandwidthSq = bandwidth * bandwidth;
		int searchOffset = (int)std::ceil(bandwidth / spatialPartitioning->cellSize);

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		auto& marks = currentPointCloud->marks["OperatorMeanShift"];
		marks.resize(numberOfPoints, 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& originalStartPos = currentPointCloud->positions[i];

				if (markedPointsOnly && marks[i] != 1)
				{
					shiftedPositions[i] = originalStartPos;
					shiftDistances[i] = 0.0f;
					return;
				}

				Eigen::Vector3f currentPos = originalStartPos;

				for (int iter = 0; iter < maxIterations; ++iter)
				{
					Eigen::Vector3f newPos = Eigen::Vector3f::Zero();
					int neighborCount = 0;

					int gx = (int)std::floor((currentPos.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
					int gy = (int)std::floor((currentPos.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
					int gz = (int)std::floor((currentPos.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

					for (int dz = -searchOffset; dz <= searchOffset; ++dz) {
						for (int dy = -searchOffset; dy <= searchOffset; ++dy) {
							for (int dx = -searchOffset; dx <= searchOffset; ++dx) {
								uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
								auto it = spatialPartitioning->voxelPointListHead.find(key);
								if (it == spatialPartitioning->voxelPointListHead.end()) continue;

								int curr = it->second;
								while (curr != -1) {
									const Eigen::Vector3f& neighborPos = currentPointCloud->positions[curr];
									float distSq = (currentPos - neighborPos).squaredNorm();

									if (distSq <= bandwidthSq) {
										newPos += neighborPos;
										neighborCount++;
									}
									curr = spatialPartitioning->nextPoint[curr];
								}
							}
						}
					}

					if (neighborCount == 0) break;

					newPos /= (float)neighborCount; // 무게 중심(Centroid)

					Eigen::Vector3f shiftVector = newPos - currentPos;

					// [방향 결정]
					if (invertDirection)
					{
						// 역방향: 무게 중심 반대쪽으로 도망감
						currentPos = currentPos - shiftVector;
					}
					else
					{
						// 정방향: 무게 중심 쪽으로 이동
						currentPos = newPos;
					}

					if (shiftVector.norm() < convergenceThreshold) {
						break;
					}
				}

				shiftedPositions[i] = currentPos;
				shiftDistances[i] = (currentPos - originalStartPos).norm();
			});

		// [Step 2] 위치 업데이트
		if (updatePositions)
		{
			// shiftedPositions에는 "마킹된 점(이동됨)"과 "마킹 안 된 점(원본 유지)"이 모두 들어있음
			// 따라서 그냥 전체를 복사하면 됨
			for (size_t i = 0; i < numberOfPoints; ++i)
			{
				currentPointCloud->positions[i] = shiftedPositions[i];
			}

			parameter.needToRebuildSpatialPartitioning = true;
			printf("MeanShift: Processed. (MarkedOnly: %s, Invert: %s)\n",
				markedPointsOnly ? "True" : "False", invertDirection ? "True" : "False");
		}

		TE(MeanShift);
	}

	void OperatorMeanShift::Visualize()
	{
		if (nullptr == cachedPointCloud || shiftedPositions.empty()) return;

		// 참고: updatePositions = true일 때는 Visualize를 호출해도 
		// 원본 포인트 자체가 이동해버렸기 때문에 구체(Sphere)와 겹쳐 보일 수 있습니다.

		size_t count = cachedPointCloud->numberOfElements;

		float maxDist = 0.0f;
		for (float d : shiftDistances) maxDist = std::max(maxDist, d);
		if (maxDist < 1e-6f) maxDist = 1.0f;

		//const auto& positions = pipeline->GetPointCloud(0)->positions;

		for (size_t i = 0; i < count; ++i)
		{
			// 이동 거리가 거의 없는 점(마킹 안 된 점 포함)은 시각화 생략하여 성능 확보
			//if (shiftDistances[i] < 1e-5f) continue;

			float t = std::clamp(shiftDistances[i] / (maxDist * visualizationScale), 0.0f, 1.0f);

			Eigen::Vector3f color;
			// Blue -> Red
			color = Eigen::Vector3f(t, 0.0f, 1.0f - t);

			Eigen::Vector3f drawPos = updatePositions ? cachedPointCloud->positions[i] : shiftedPositions[i];

			//drawPos = positions[i];

			VD::AddSphere(
				"MeanShiftResult",
				drawPos,
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
			);
		}
	}
#pragma endregion


#pragma region OperatorPointCloudLaplacianSmoothing
	OperatorPointCloudLaplacianSmoothing::OperatorPointCloudLaplacianSmoothing(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorPointCloudLaplacianSmoothing::OperatorPointCloudLaplacianSmoothing::Process()
	{
		TS(LaplacianSmoothing);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		std::vector<Eigen::Vector3f> nextPositions = currentPointCloud->positions;

		auto& marks = currentPointCloud->marks["OperatorPointCloudLaplacianSmoothing"];
		marks.resize(numberOfPoints, 0);

		for (int iter = 0; iter < iterations; ++iter)
		{
			std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
				{
					if (preserveMarks && !marks.empty() && marks[i] != 0)
					{
						nextPositions[i] = currentPointCloud->positions[i];
						return;
					}

					const Eigen::Vector3f& p = currentPointCloud->positions[i];
					Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
					int neighborCount = 0;

					int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
					int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
					int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

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
										if ((p - currentPointCloud->positions[curr]).squaredNorm() <= searchRadiusSq)
										{
											centroid += currentPointCloud->positions[curr];
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
						Eigen::Vector3f delta = centroid - p;

						nextPositions[i] = p + delta * smoothingFactor;
					}
					else
					{
						nextPositions[i] = p;
					}
				});

			currentPointCloud->positions = nextPositions;
		}

		TE(LaplacianSmoothing);
	}

	void OperatorPointCloudLaplacianSmoothing::Visualize()
	{
		if (nullptr == cachedPointCloud) return;

		size_t count = cachedPointCloud->numberOfElements;
		for (size_t i = 0; i < count; ++i)
		{
			VD::AddSphere(
				"SmoothedPoints",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f)
			);
		}
	}
#pragma endregion

#pragma region OperatorKNNSmoothing
	OperatorKNNSmoothing::OperatorKNNSmoothing(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorKNNSmoothing::Process()
	{
		TS(KNNSmoothing);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		std::vector<Eigen::Vector3f> nextPositions = currentPointCloud->positions;

		auto& marks = currentPointCloud->marks["OperatorKNNSmoothing"];
		marks.resize(numberOfPoints, 0);

		for (int iter = 0; iter < iterations; ++iter)
		{
			std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
				{
					if (false == (preserveMarks && !marks.empty() && marks[i] != 0))
					{
						return;
					}

					const Eigen::Vector3f& p = currentPointCloud->positions[i];

					currentPointCloud->colors[i] = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

					std::vector<unsigned int> neighborIndices;
					std::vector<float> neighborDistances;
					neighborIndices.reserve(kNeighbors);
					neighborDistances.reserve(kNeighbors);

					spatialPartitioning->GetKNearestNeighbors(
						currentPointCloud->positions,
						p,
						kNeighbors,
						neighborIndices,
						neighborDistances
					);

					if (!neighborIndices.empty())
					{
						Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
						float validCount = 0.0f;

						for (unsigned int idx : neighborIndices)
						{
							centroid += currentPointCloud->positions[idx];
							validCount += 1.0f;
						}

						if (validCount > 0.0f)
						{
							centroid /= validCount;
							Eigen::Vector3f delta = centroid - p;

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

			currentPointCloud->positions = nextPositions;
		}

		TE(KNNSmoothing);
	}

	void OperatorKNNSmoothing::Visualize()
	{
		if (nullptr == cachedPointCloud) return;

		size_t count = cachedPointCloud->numberOfElements;
		for (size_t i = 0; i < count; ++i)
		{
			VD::AddSphere(
				"KNNSmoothedPoints",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f)
			);
		}
	}
#pragma endregion

#pragma region OperatorSurfaceFitting
	OperatorSurfaceFitting::OperatorSurfaceFitting(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorSurfaceFitting::Process()
	{
		TS(SurfaceFitting);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		for (int i = 0; i < iteration; i++)
		{
			std::vector<Eigen::Vector3f> newPositions = currentPointCloud->positions;
			std::vector<Eigen::Vector3f> newNormals = currentPointCloud->normals;

			std::vector<int> indices(numberOfPoints);
			std::iota(indices.begin(), indices.end(), 0);

			std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
				{
					const Eigen::Vector3f& p = currentPointCloud->positions[i];

					std::vector<unsigned int> neighborIndices;
					std::vector<float> neighborDistances;
					neighborIndices.reserve(kNeighbors);
					neighborDistances.reserve(kNeighbors);

					spatialPartitioning->GetKNearestNeighbors(
						currentPointCloud->positions,
						p,
						kNeighbors,
						neighborIndices,
						neighborDistances
					);

					size_t k = neighborIndices.size();
					if (k < 4) return;

					Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
					float totalWeight = 0.0f;

					float maxDist = neighborDistances.back();
					float h = std::max(maxDist * 0.5f, 1e-6f);
					float hSq = h * h;

					std::vector<float> weights(k);

					for (size_t j = 0; j < k; ++j)
					{
						float distSq = neighborDistances[j] * neighborDistances[j];
						float w = std::exp(-distSq / hSq);

						weights[j] = w;
						centroid += currentPointCloud->positions[neighborIndices[j]] * w;
						totalWeight += w;
					}

					if (totalWeight < 1e-6f) return;
					centroid /= totalWeight;

					float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

					for (size_t j = 0; j < k; ++j)
					{
						Eigen::Vector3f r = currentPointCloud->positions[neighborIndices[j]] - centroid;
						float w = weights[j];

						xx += w * r.x() * r.x();
						xy += w * r.x() * r.y();
						xz += w * r.x() * r.z();
						yy += w * r.y() * r.y();
						yz += w * r.y() * r.z();
						zz += w * r.z() * r.z();
					}

					Eigen::Matrix3f cov;
					cov(0, 0) = xx; cov(0, 1) = xy; cov(0, 2) = xz;
					cov(1, 0) = xy; cov(1, 1) = yy; cov(1, 2) = yz;
					cov(2, 0) = xz; cov(2, 1) = yz; cov(2, 2) = zz;

					cov /= totalWeight;

					Eigen::Vector3f eigenVals;
					Eigen::Matrix3f eigenVecs;
					ComputeEigenDecomposition(cov, eigenVals, eigenVecs);

					Eigen::Vector3f planeNormal = eigenVecs.col(0);

					if (planeNormal.dot(currentPointCloud->normals[i]) < 0.0f)
					{
						planeNormal = -planeNormal;
					}

					Eigen::Vector3f diff = p - centroid;
					float distToPlane = diff.dot(planeNormal);

					newPositions[i] = p - planeNormal * distToPlane;

					if (updateNormals)
					{
						newNormals[i] = planeNormal;
					}

					if (0.1f < (newPositions[i] - p).norm())
					{
						currentPointCloud->colors[i] = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
					}
				});

			currentPointCloud->positions = newPositions;
			if (updateNormals)
			{
				currentPointCloud->normals = newNormals;
			}
		}

		TE(SurfaceFitting);
	}

	void OperatorSurfaceFitting::Visualize()
	{
		if (nullptr == cachedPointCloud) return;

		size_t count = cachedPointCloud->numberOfElements;
		for (size_t i = 0; i < count; ++i)
		{
			VD::AddSphere(
				"FittedPoints",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f)
			);
		}
	}

	void OperatorSurfaceFitting::ComputeEigenDecomposition(const Eigen::Matrix3f& cov, Eigen::Vector3f& outEvals, Eigen::Matrix3f& outEvecs)
	{
		double m = (cov(0, 0) + cov(1, 1) + cov(2, 2)) / 3.0;
		double p = (std::pow(cov(0, 0) - m, 2.0) + std::pow(cov(1, 1) - m, 2.0) + std::pow(cov(2, 2) - m, 2.0) +
			2.0 * (std::pow(cov(0, 1), 2.0) + std::pow(cov(0, 2), 2.0) + std::pow(cov(1, 2), 2.0))) / 6.0;

		double q = (cov - Eigen::Matrix3f::Identity() * m).determinant() / 2.0;
		double phi = 0.0;
		if (p > 1e-12) phi = std::atan2(std::sqrt(std::max(0.0, p * p * p - q * q)), q) / 3.0;
		if (phi < 0) phi += 3.14159265358979323846 / 3.0;

		double eig1 = m + 2.0 * std::sqrt(p) * std::cos(phi);
		double eig2 = m + 2.0 * std::sqrt(p) * std::cos(phi + 2.0 * 3.14159265358979323846 / 3.0);
		double eig3 = 3.0 * m - eig1 - eig2;

		outEvals = Eigen::Vector3f((float)eig1, (float)eig2, (float)eig3);

		int i0 = 0, i1 = 1, i2 = 2;
		if (outEvals[i0] > outEvals[i1]) std::swap(i0, i1);
		if (outEvals[i1] > outEvals[i2]) std::swap(i1, i2);
		if (outEvals[i0] > outEvals[i1]) std::swap(i0, i1);

		Eigen::Vector3f sortedEvals;
		sortedEvals.x() = outEvals[i0];
		sortedEvals.y() = outEvals[i1];
		sortedEvals.z() = outEvals[i2];
		outEvals = sortedEvals;

		auto computeVec = [&](float lambda) -> Eigen::Vector3f {
			Eigen::Matrix3f A = cov - Eigen::Matrix3f::Identity() * lambda;

			Eigen::Vector3f r0 = A.row(0);
			Eigen::Vector3f r1 = A.row(1);
			Eigen::Vector3f r2 = A.row(2);

			Eigen::Vector3f v1 = r0.cross(r1);
			Eigen::Vector3f v2 = r1.cross(r2);
			Eigen::Vector3f v3 = r2.cross(r0);

			float l1 = v1.dot(v1);
			float l2 = v2.dot(v2);
			float l3 = v3.dot(v3);

			Eigen::Vector3f maxV = v1;
			if (l2 > l1) maxV = v2;
			if (l3 > std::max(l1, l2)) maxV = v3;

			if (maxV.norm() > 1e-6f) return maxV.normalized();

			return Eigen::Vector3f(1, 0, 0);
			};

		outEvecs.col(0) = computeVec(outEvals.x());
		outEvecs.col(1) = computeVec(outEvals.y());
		outEvecs.col(2) = outEvecs.col(0).cross(outEvecs.col(1)).normalized();
	}
#pragma endregion

#pragma region OperatorPointDensity
	OperatorPointDensity::OperatorPointDensity(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorPointDensity::Process()
	{
		TS(PointDensity);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;
		densities.resize(numberOfPoints);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusScale;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				int neighborCount = 0;

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

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
									if ((p - currentPointCloud->positions[currIdx]).squaredNorm() <= searchRadiusSq)
									{
										neighborCount++;
									}
								}
								currIdx = spatialPartitioning->nextPoint[currIdx];
							}
						}
					}
				}

				densities[i] = (float)neighborCount;
			});

		if (numberOfPoints > 0)
		{
			auto result = std::minmax_element(densities.begin(), densities.end());
			minDensity = *result.first;
			maxDensity = *result.second;
		}

		TE(PointDensity);
	}

	void OperatorPointDensity::Visualize()
	{
		if (nullptr == cachedPointCloud || densities.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;
		float range = maxDensity - minDensity;
		if (range < 0.0001f) range = 1.0f;

		for (size_t i = 0; i < count; ++i)
		{
			float val = (densities[i] - minDensity) / range;

			Eigen::Vector3f color;
			if (val < 0.5f)
			{
				color = Eigen::Vector3f(1, 0, 0) * (1.0f - val * 2.0f) + Eigen::Vector3f(0, 1, 0) * (val * 2.0f);
			}
			else
			{
				float t = (val - 0.5f) * 2.0f;
				color = Eigen::Vector3f(0, 1, 0) * (1.0f - t) + Eigen::Vector3f(0, 0, 1) * t;
			}

			VD::AddSphere(
				"PointDensity",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius * (2.0f - val),
				Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
			);
		}
	}
#pragma endregion

#pragma region OperatorFindOverlappingPoints
	OperatorFindOverlappingPoints::OperatorFindOverlappingPoints(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorFindOverlappingPoints::Process()
	{
		TS(FindOverlappingPoints);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		{
			overlapDistanceThreshold = parameter.GetParameter<float>("overlapDistanceThreshold", overlapDistanceThreshold);

			maxNormalAngle = parameter.GetParameter<float>("maxNormalAngle", maxNormalAngle);
		}

		float internalCosThreshold = std::cos(maxNormalAngle * 3.14159265f / 180.0f);

		currentPointCloud->marks["OperatorFindOverlappingPoints"].assign(numberOfPoints, 0);
		overlappingCount = 0;

		float thresholdSq = overlapDistanceThreshold * overlapDistanceThreshold;

		auto& marks = currentPointCloud->marks["OperatorFindOverlappingPoints"];
		marks.resize(numberOfPoints, 0);

		std::for_each(std::execution::par, spatialPartitioning->voxelPointListHead.begin(), spatialPartitioning->voxelPointListHead.end(),
			[&](const auto& cell)
			{
				uint64_t key = cell.first;
				int headIdx = cell.second;

				int gx = (int)(key >> 42);
				int gy = (int)((key >> 21) & 0x1FFFFF);
				int gz = (int)(key & 0x1FFFFF);

				for (int curr = headIdx; curr != -1; curr = spatialPartitioning->nextPoint[curr])
				{
					const Eigen::Vector3f& pA = currentPointCloud->positions[curr];
					const Eigen::Vector3f& nA = currentPointCloud->normals[curr];

					for (int dz = -1; dz <= 1; ++dz) {
						for (int dy = -1; dy <= 1; ++dy) {
							for (int dx = -1; dx <= 1; ++dx) {

								uint64_t neighborKey = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
								auto it = spatialPartitioning->voxelPointListHead.find(neighborKey);
								if (it == spatialPartitioning->voxelPointListHead.end()) continue;

								for (int other = it->second; other != -1; other = spatialPartitioning->nextPoint[other])
								{
									if (curr == other) continue;

									if (curr < other)
									{
										const Eigen::Vector3f& pB = currentPointCloud->positions[other];

										if ((pA - pB).squaredNorm() <= thresholdSq)
										{
											const Eigen::Vector3f& nB = currentPointCloud->normals[other];

											if (nA.dot(nB) >= internalCosThreshold)
											{
												marks[other] = 1;
											}
										}
									}
								}
							}
						}
					}
				}
			});

		for (int m : currentPointCloud->marks["OperatorFindOverlappingPoints"]) {
			if (m == 1) overlappingCount++;
		}

		TE(FindOverlappingPoints);
	}

	void OperatorFindOverlappingPoints::Visualize()
	{
		if (nullptr == cachedPointCloud) return;

		size_t count = cachedPointCloud->numberOfElements;

		for (size_t i = 0; i < count; ++i)
		{
			if (cachedPointCloud->marks["OperatorFindOverlappingPoints"][i] == 1)
			{
				VD::AddSphere(
					"OverlappingPoints",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius * 1.2f,
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f)
				);
			}
			else
			{
				VD::AddSphere(
					"UniquePoints",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.1f)
				);
			}
		}
	}
#pragma endregion

#pragma region Morphological
#pragma region MorphologyHelper
	void MorphologyHelper::ApplyMorphology(
		std::shared_ptr<PointCloud> cloud,
		SparseGrid* grid,
		float radius,
		int mode,
		int neighborSearchOffset)
	{
		if (cloud->numberOfElements == 0) return;

		std::vector<Eigen::Vector3f> newPositions = cloud->positions;
		float radiusSq = radius * radius;

		std::vector<int> indices(cloud->numberOfElements);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = cloud->positions[i];
				const Eigen::Vector3f& n = cloud->normals[i];

				float targetH = 0.0f;
				bool found = false;

				int gx = (int)std::floor((p.x() - grid->aabb.min.x()) / grid->cellSize);
				int gy = (int)std::floor((p.y() - grid->aabb.min.y()) / grid->cellSize);
				int gz = (int)std::floor((p.z() - grid->aabb.min.z()) / grid->cellSize);

				for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz) {
					for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy) {
						for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx) {
							uint64_t key = grid->GetKey(gx + dx, gy + dy, gz + dz);
							auto it = grid->voxelPointListHead.find(key);
							if (it == grid->voxelPointListHead.end()) continue;

							int curr = it->second;
							while (curr != -1) {
								if (curr != i) {
									Eigen::Vector3f diff = cloud->positions[curr] - p;
									if (diff.squaredNorm() <= radiusSq) {
										// Projection Height
										float h = diff.dot(n);
										if (mode == 0) { // Erosion
											if (h < targetH) targetH = h;
										}
										else { // Dilation
											if (h > targetH) targetH = h;
										}
										found = true;
									}
								}
								curr = grid->nextPoint[curr];
							}
						}
					}
				}

				if (found && std::abs(targetH) > 1e-6f) {
					newPositions[i] = p + n * targetH;
				}
			});

		cloud->positions = newPositions;
	}

	void MorphologyHelper::MarkOverlappingPointsAbove(
		std::shared_ptr<PointCloud> cloud, SparseGrid* grid, float overlapDistThreshold, float maxAngle)
	{
		float thresholdSq = overlapDistThreshold * overlapDistThreshold;
		float minCos = std::cos(maxAngle * 3.14159265f / 180.0f);

		auto& marks = cloud->marks["OperatorFindOverlappingPoints"];
		marks.assign(cloud->numberOfElements, 0);

		std::for_each(std::execution::par, grid->voxelPointListHead.begin(), grid->voxelPointListHead.end(),
			[&](const auto& cell)
			{
				uint64_t key = cell.first;
				int headIdx = cell.second;
				int gx = (int)(key >> 42);
				int gy = (int)((key >> 21) & 0x1FFFFF);
				int gz = (int)(key & 0x1FFFFF);

				for (int curr = headIdx; curr != -1; curr = grid->nextPoint[curr])
				{
					const Eigen::Vector3f& pA = cloud->positions[curr];
					const Eigen::Vector3f& nA = cloud->normals[curr];

					for (int dz = -1; dz <= 1; ++dz) {
						for (int dy = -1; dy <= 1; ++dy) {
							for (int dx = -1; dx <= 1; ++dx) {
								uint64_t nKey = grid->GetKey(gx + dx, gy + dy, gz + dz);
								auto it = grid->voxelPointListHead.find(nKey);
								if (it == grid->voxelPointListHead.end()) continue;

								for (int other = it->second; other != -1; other = grid->nextPoint[other])
								{
									if (curr == other) continue;

									if (curr < other)
									{
										Eigen::Vector3f diff = cloud->positions[other] - pA;
										if (diff.squaredNorm() <= thresholdSq)
										{
											if (nA.dot(cloud->normals[other]) >= minCos)
											{
												float heightDiff = diff.dot(nA);

												if (heightDiff > 0)
												{
													marks[other] = 1;
												}
												else
												{
													marks[curr] = 1;
												}
											}
										}
									}
								}
							}
						}
					}
				}
			});
	}
#pragma endregion
	
#pragma region OperatorApplyMorphology
	OperatorApplyMorphology::OperatorApplyMorphology(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorApplyMorphology::Process()
	{
		TS(ApplyMorphology);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning) {
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}
		cachedPointCloud = currentPointCloud;

		{
			operation = parameter.GetParameter<std::string>("operation", "Erosion"); // 기본값 Erosion
			radius = parameter.GetParameter<float>("radius", radius);
			overlapDist = parameter.GetParameter<float>("overlapDist", overlapDist);
			maxAngle = parameter.GetParameter<float>("maxAngle", maxAngle);
			markingOnly = parameter.GetParameter<bool>("markingOnly", markingOnly);
		}

		std::vector<Eigen::Vector3f> originalPositions;
		if (markingOnly)
		{
			originalPositions = currentPointCloud->positions;
		}

		if (operation == "Erosion")
		{
			MorphologyHelper::ApplyMorphology(currentPointCloud, spatialPartitioning, radius, 0);
		}
		else if (operation == "Dilation")
		{
			MorphologyHelper::ApplyMorphology(currentPointCloud, spatialPartitioning, radius, 1);
		}
		else if (operation == "ErosionDilation")
		{
			MorphologyHelper::ApplyMorphology(currentPointCloud, spatialPartitioning, radius, 0);
			MorphologyHelper::ApplyMorphology(currentPointCloud, spatialPartitioning, radius, 1);
		}
		else if (operation == "DilationErosion")
		{
			MorphologyHelper::ApplyMorphology(currentPointCloud, spatialPartitioning, radius, 1);
			MorphologyHelper::ApplyMorphology(currentPointCloud, spatialPartitioning, radius, 0);
		}

		if (spatialPartitioning != nullptr)
		{
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
		}

		MorphologyHelper::MarkOverlappingPointsAbove(currentPointCloud, spatialPartitioning, overlapDist, maxAngle);

		if (markingOnly)
		{
			currentPointCloud->positions = originalPositions;

			if (spatialPartitioning != nullptr)
			{
				spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			}
		}

		TE(ApplyMorphology);
	}

	void OperatorApplyMorphology::Visualize()
	{
		if (nullptr == cachedPointCloud) return;

		size_t count = cachedPointCloud->numberOfElements;
		for (size_t i = 0; i < count; ++i)
		{
			if (cachedPointCloud->marks["OperatorApplyMorphology"][i] == 1)
			{
				VD::AddSphere(
					"Morphology_Marked_Removed",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius * 1.2f,
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f)
				);
			}
			else
			{
				VD::AddSphere(
					"Morphology_Marked_Kept",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.2f)
				);
			}
		}
	}
#pragma endregion

#pragma region OperatorErosionAndClustering
	OperatorErosionAndClustering::OperatorErosionAndClustering(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorErosionAndClustering::Process()
	{
		TS(ErosionAndClustering);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		// 파라미터 로드
		{
			erosionRadius = parameter.GetParameter<float>("erosionRadius", 0.05f); // 1회당 깎는 양 (작게 설정)
			erosionIterations = parameter.GetParameter<int>("erosionIterations", 5); // 반복 횟수 (많이 설정)
			clusterDistance = parameter.GetParameter<float>("clusterDistance", 0.06f); // erosionRadius와 비슷하거나 약간 크게
			minClusterSize = parameter.GetParameter<int>("minClusterSize", 50);
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", 3);
			visualizeErodedOnly = parameter.GetParameter<bool>("visualizeErodedOnly", false); // 디버깅용
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;
		erodedPositions = currentPointCloud->positions; // 초기값은 원본

		float erosionRadiusSq = erosionRadius * erosionRadius;
		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		// [Step 1] 반복적 침식 (Iterative Erosion)
		// 한 번에 확 깎지 않고, 조금씩 여러 번 깎아야 형상이 뭉개지지 않고 틈새만 벌어집니다.
		for (int iter = 0; iter < erosionIterations; ++iter)
		{
			// 매 반복마다 현재 erodedPositions를 기준으로 그리드 빌드 (정확도 향상)
			SparseGrid iterGrid;
			PointCloud iterCloud;
			iterCloud.numberOfElements = numberOfPoints;
			iterCloud.positions = erodedPositions;
			for (const auto& p : erodedPositions) iterCloud.aabb.Expand(p);
			iterGrid.Build(iterCloud, std::max(erosionRadius * 3.0f, Configuration::voxelSize));

			std::vector<Eigen::Vector3f> nextPositions = erodedPositions;

			std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
				{
					const Eigen::Vector3f& p = erodedPositions[i];
					const Eigen::Vector3f& n = currentPointCloud->normals[i]; // 법선은 원본 유지 (안정성)

					float minH = 0.0f;
					bool found = false;

					int gx = (int)std::floor((p.x() - iterGrid.aabb.min.x()) / iterGrid.cellSize);
					int gy = (int)std::floor((p.y() - iterGrid.aabb.min.y()) / iterGrid.cellSize);
					int gz = (int)std::floor((p.z() - iterGrid.aabb.min.z()) / iterGrid.cellSize);

					for (int dz = -1; dz <= 1; ++dz) {
						for (int dy = -1; dy <= 1; ++dy) {
							for (int dx = -1; dx <= 1; ++dx) {
								uint64_t key = iterGrid.GetKey(gx + dx, gy + dy, gz + dz);
								auto it = iterGrid.voxelPointListHead.find(key);
								if (it == iterGrid.voxelPointListHead.end()) continue;

								int curr = it->second;
								while (curr != -1) {
									if (curr != i) {
										Eigen::Vector3f diff = erodedPositions[curr] - p;
										if (diff.squaredNorm() <= erosionRadiusSq) {
											float h = diff.dot(n);
											if (h < minH) minH = h; // 가장 깊은 곳 찾기
											found = true;
										}
									}
									curr = iterGrid.nextPoint[curr];
								}
							}
						}
					}

					if (found && minH < -1e-6f)
					{
						nextPositions[i] = p + n * minH;
					}
				});

			erodedPositions = nextPositions;
		}

		if (visualizeErodedOnly)
		{
			// 디버깅 모드: 클러스터링 안 하고 깎인 모습만 보여주고 종료
			alog("Erosion Only Mode. Check if gaps are visible.\n");
			TE(ErosionAndClustering);
			return;
		}

		// [Step 2] 침식된 좌표를 기준으로 클러스터링 수행
		SparseGrid clusterGrid;
		PointCloud tempCloud;
		tempCloud.numberOfElements = numberOfPoints;
		tempCloud.positions = erodedPositions;
		for (const auto& p : erodedPositions) tempCloud.aabb.Expand(p);
		clusterGrid.Build(tempCloud, clusterDistance * 2.0f);

		AtomicDisjointSet dsu;
		dsu.Initialize(numberOfPoints);

		float clusterDistSq = clusterDistance * clusterDistance;

		struct CellData { uint64_t key; int headIdx; };
		std::vector<CellData> flatCells;
		flatCells.reserve(clusterGrid.voxelPointListHead.size());
		for (const auto& pair : clusterGrid.voxelPointListHead) flatCells.push_back({ pair.first, pair.second });

		const uint64_t mask = 0x1FFFFF;

		std::for_each(std::execution::par, flatCells.begin(), flatCells.end(), [&](const CellData& cell)
			{
				uint64_t key = cell.key;
				int gx = (int)(key >> 42);
				int gy = (int)((key >> 21) & mask);
				int gz = (int)(key & mask);

				for (int i = cell.headIdx; i != -1; i = clusterGrid.nextPoint[i])
				{
					const Eigen::Vector3f& pA = erodedPositions[i];

					// 인접 셀 탐색
					for (int dz = -1; dz <= 1; ++dz) {
						for (int dy = -1; dy <= 1; ++dy) {
							for (int dx = -1; dx <= 1; ++dx) {
								uint64_t nKey = clusterGrid.GetKey(gx + dx, gy + dy, gz + dz);
								auto it = clusterGrid.voxelPointListHead.find(nKey);
								if (it == clusterGrid.voxelPointListHead.end()) continue;

								for (int j = it->second; j != -1; j = clusterGrid.nextPoint[j])
								{
									if (i < j) // 중복 방지
									{
										if ((pA - erodedPositions[j]).squaredNorm() <= clusterDistSq)
										{
											dsu.Union(i, j);
										}
									}
								}
							}
						}
					}
				}
			});

		// [Step 3] ID 정리
		std::map<int, int> rootToClusterID;
		std::map<int, int> clusterSize;
		int currentClusterCount = 0;

		std::vector<int> roots(numberOfPoints);
		for (size_t i = 0; i < numberOfPoints; ++i) {
			roots[i] = dsu.Find((int)i);
			clusterSize[roots[i]]++;
		}

		if (currentPointCloud->pointClusterIDs.size() != numberOfPoints)
			currentPointCloud->pointClusterIDs.resize(numberOfPoints);

		for (auto const& [root, size] : clusterSize) {
			if (size >= minClusterSize) {
				rootToClusterID[root] = currentClusterCount++;
			}
		}

		for (size_t i = 0; i < numberOfPoints; ++i) {
			int root = roots[i];
			if (rootToClusterID.find(root) != rootToClusterID.end()) {
				currentPointCloud->pointClusterIDs[i] = rootToClusterID[root];
			}
			else {
				currentPointCloud->pointClusterIDs[i] = -1;
			}
		}

		alog("Erosion(Iter:%d) & Clustering Done. Clusters: %d\n", erosionIterations, currentClusterCount);
		TE(ErosionAndClustering);
	}

	void OperatorErosionAndClustering::Visualize()
	{
		if (nullptr == cachedPointCloud) return;

		// 디버깅 모드: 깎인 점들을 직접 보여줌 (공중에 떠있는 점들)
		if (visualizeErodedOnly)
		{
			for (size_t i = 0; i < erodedPositions.size(); ++i)
			{
				VD::AddSphere(
					"ErodedDebug",
					erodedPositions[i],
					Configuration::pointVisualizationRadius * 0.8f,
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 0.5f) // 빨간색 반투명
				);
			}
			return;
		}

		// 정상 모드: 클러스터링 결과
		auto contrastingColors = Color::GetContrastingColorsWithoutBWRGB(256);
		for (size_t i = 0; i < cachedPointCloud->numberOfElements; ++i)
		{
			int clusterId = cachedPointCloud->pointClusterIDs[i];
			if (clusterId < 0) continue;

			auto color = contrastingColors[clusterId % contrastingColors.size()];
			VD::AddSphere(
				"ErosionClustered",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(color.r, color.g, color.b, 1.0f)
			);
		}
	}
#pragma endregion
#pragma endregion

#pragma region About Normal
#pragma region OperatorNormalDeviation
	OperatorNormalDeviation::OperatorNormalDeviation(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorNormalDeviation::Process()
	{
		TS(NormalDeviation);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		{ // Apply parameters
			searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
			maxDeviationAngle = parameter.GetParameter<float>("maxDeviationAngle", maxDeviationAngle);
		}

		deviations.resize(numberOfPoints);
		currentPointCloud->marks["OperatorNormalDeviation"].assign(numberOfPoints, 0);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				const Eigen::Vector3f& n = currentPointCloud->normals[i];

				Eigen::Vector3f neighborNormalSum = Eigen::Vector3f::Zero();
				int neighborCount = 0;

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

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
									if ((p - currentPointCloud->positions[curr]).squaredNorm() <= searchRadiusSq)
									{
										if (n.dot(currentPointCloud->normals[curr]) >= 0.0f)
										{
											neighborNormalSum += currentPointCloud->normals[curr];
										}
										else
										{
											neighborNormalSum -= currentPointCloud->normals[curr];
										}
										neighborCount++;
									}
								}
								curr = spatialPartitioning->nextPoint[curr];
							}
						}
					}
				}

				float angleDeg = 0.0f;
				if (neighborCount > 0)
				{
					neighborNormalSum.normalize();
					float dot = n.dot(neighborNormalSum);
					dot = std::clamp(dot, -1.0f, 1.0f);
					angleDeg = std::acos(dot) * 180.0f / 3.14159265f;
				}

				deviations[i] = angleDeg;

				if (angleDeg > maxDeviationAngle)
				{
					currentPointCloud->marks["OperatorNormalDeviation"][i] = 1;
				}
			});

		TE(NormalDeviation);
	}

	void OperatorNormalDeviation::Visualize()
	{
		VisualizeDefault();
	}

	void OperatorNormalDeviation::VisualizeDefault()
	{
		if (nullptr == cachedPointCloud || deviations.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		for (size_t i = 0; i < count; ++i)
		{
			float val = deviations[i];

			if (cachedPointCloud->marks["OperatorNormalDeviation"][i] == 1)
			{
				// Highlight detected outliers
				VD::AddSphere(
					"HighDeviationPoints",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius * 1.5f,
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f) // Red
				);
			}
			else
			{
				float t = std::clamp(val / maxDeviationAngle, 0.0f, 1.0f);
				Eigen::Vector3f color = Eigen::Vector3f(0.0f, 0.0f, 1.0f) * (1.0f - t) + Eigen::Vector3f(0.0f, 1.0f, 1.0f) * t;

				VD::AddSphere(
					"NormalPoints",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(color.x(), color.y(), color.z(), 0.3f)
				);
			}
		}
	}

	void OperatorNormalDeviation::VisualizeHeatmap()
	{
		if (nullptr == cachedPointCloud || deviations.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		for (size_t i = 0; i < count; ++i)
		{
			float val = deviations[i];
			float t = std::clamp(val / maxDeviationAngle, 0.0f, 1.0f);
			Eigen::Vector3f color;
			if (t < 0.25f)
			{
				float localT = t / 0.25f;
				color = Eigen::Vector3f(0.0f, localT, 1.0f);
			}
			else if (t < 0.5f)
			{
				float localT = (t - 0.25f) / 0.25f;
				color = Eigen::Vector3f(0.0f, 1.0f, 1.0f - localT);
			}
			else if (t < 0.75f)
			{
				float localT = (t - 0.5f) / 0.25f;
				color = Eigen::Vector3f(localT, 1.0f, 0.0f);
			}
			else
			{
				float localT = (t - 0.75f) / 0.25f;
				color = Eigen::Vector3f(1.0f, 1.0f - localT, 0.0f);
			}

			VD::AddSphere(
				"NormalDeviationHeatmap",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
			);
		}
	}
#pragma endregion
	
#pragma region OperatorNormalGradient
	OperatorNormalGradient::OperatorNormalGradient(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorNormalGradient::Process()
	{
		TS(NormalGradient);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		{
			searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
			useAlphaGradient = parameter.GetParameter<bool>("useAlphaGradient", useAlphaGradient);
			visualizationSigma = parameter.GetParameter<float>("visualizationSigma", visualizationSigma);
		}

		gradients.assign(numberOfPoints, 0.0f);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				const Eigen::Vector3f& n_i = currentPointCloud->normals[i];

				float diffSum = 0.0f;
				float weightSum = 0.0f;

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

				for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz) {
					for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy) {
						for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx) {
							uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
							auto it = spatialPartitioning->voxelPointListHead.find(key);
							if (it == spatialPartitioning->voxelPointListHead.end()) continue;

							int curr = it->second;
							while (curr != -1) {
								if (curr != i) {
									Eigen::Vector3f diff = currentPointCloud->positions[curr] - p;
									float distSq = diff.squaredNorm();

									if (distSq <= searchRadiusSq && distSq > 1e-8f) {
										float dist = std::sqrt(distSq);
										float dot = n_i.dot(currentPointCloud->normals[curr]);
										float normalDiff = 1.0f - std::clamp(dot, -1.0f, 1.0f);
										float weight = 1.0f / dist;

										diffSum += normalDiff * weight;
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
					gradients[i] = diffSum / weightSum;
				}
			});

		double sum = 0.0;
		double sqSum = 0.0;
		for (float v : gradients)
		{
			sum += v;
			sqSum += v * v;
		}
		gradientMean = (float)(sum / numberOfPoints);
		double variance = (sqSum / numberOfPoints) - (gradientMean * gradientMean);
		gradientStdDev = (float)std::sqrt(std::max(0.0, variance));

		TE(NormalGradient);
	}

	void OperatorNormalGradient::Visualize()
	{
		if (nullptr == cachedPointCloud || gradients.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		// Auto-Exposure Threshold Calculation
		// 평균 + (표준편차 * Sigma)를 최대값(Red)으로 설정
		float maxThreshold = gradientMean + gradientStdDev * visualizationSigma;

		float minThreshold = gradientMean * 0.5f;

		float range = maxThreshold - minThreshold;
		if (range < 1e-6f) range = 1.0f;

		for (size_t i = 0; i < count; ++i)
		{
			float val = gradients[i];

			float clampedVal = std::clamp(val, minThreshold, maxThreshold);

			float t = (clampedVal - minThreshold) / range;

			Eigen::Vector3f color;

			if (t < 0.25f)
			{
				float localT = t / 0.25f;
				color = Eigen::Vector3f(0.0f, localT, 1.0f);
			}
			else if (t < 0.5f)
			{
				float localT = (t - 0.25f) / 0.25f;
				color = Eigen::Vector3f(0.0f, 1.0f, 1.0f - localT);
			}
			else if (t < 0.75f)
			{
				float localT = (t - 0.5f) / 0.25f;
				color = Eigen::Vector3f(localT, 1.0f, 0.0f);
			}
			else
			{
				float localT = (t - 0.75f) / 0.25f;
				color = Eigen::Vector3f(1.0f, 1.0f - localT, 0.0f);
			}

			VD::AddSphere(
				"NormalGradientMap",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(color.x(), color.y(), color.z(), useAlphaGradient ? t : 1.0f)
			);
		}
	}
#pragma endregion

#pragma region OperatorNormalVariance
	OperatorNormalVariance::OperatorNormalVariance(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorNormalVariance::Process()
	{
		TS(NormalVariance);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		// [필수] 법선 데이터 확인
		if (currentPointCloud->normals.empty())
		{
			printf("[Error] OperatorNormalVariance: Normals are empty. Run LocalPlaneFitting first.\n");
			return;
		}

		// 공간 분할 구조 빌드
		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		// 파라미터 로드
		{
			searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
			visualizationSigma = parameter.GetParameter<float>("visualizationSigma", visualizationSigma);
		}

		normalVariances.assign(numberOfPoints, 0.0f);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		// [Step 1] 병렬 처리: 법선 분산 계산
		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];

				// 법선 합계를 저장할 벡터
				Eigen::Vector3f normalSum = Eigen::Vector3f::Zero();
				int neighborCount = 0;

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

				for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz) {
					for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy) {
						for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx) {

							uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
							auto it = spatialPartitioning->voxelPointListHead.find(key);
							if (it == spatialPartitioning->voxelPointListHead.end()) continue;

							int curr = it->second;
							while (curr != -1) {
								// 자기 자신도 포함해서 분산을 계산하는 것이 일반적임 (또는 제외 가능)
								float distSq = (p - currentPointCloud->positions[curr]).squaredNorm();

								if (distSq <= searchRadiusSq) {
									normalSum += currentPointCloud->normals[curr];
									neighborCount++;
								}
								curr = spatialPartitioning->nextPoint[curr];
							}
						}
					}
				}

				if (neighborCount > 1)
				{
					// 평균 법선 벡터 계산
					Eigen::Vector3f meanNormal = normalSum / (float)neighborCount;

					// [핵심 로직] 단위 벡터들의 분산 = 1 - ||MeanNormal||^2
					// 1.0 (최대 분산, 완전 상쇄) ~ 0.0 (최소 분산, 완전 일치)
					float meanLengthSq = meanNormal.squaredNorm();

					// 부동소수점 오차로 1.0을 살짝 넘는 경우 방지
					if (meanLengthSq > 1.0f) meanLengthSq = 1.0f;

					normalVariances[i] = 1.0f - meanLengthSq;
				}
				else
				{
					normalVariances[i] = 0.0f; // 이웃이 없거나 혼자면 분산 0
				}
			});

		// [Step 2] 통계 계산
		double sum = 0.0;
		double sqSum = 0.0;
		for (float v : normalVariances)
		{
			sum += v;
			sqSum += v * v;
		}
		varianceMean = (float)(sum / numberOfPoints);
		double totalVar = (sqSum / numberOfPoints) - (varianceMean * varianceMean);
		varianceStdDev = std::sqrt(std::max(0.0, totalVar));

		{
			auto& marks = currentPointCloud->marks["OperatorNormalVariance"];
			marks.resize(numberOfPoints);

			size_t count = cachedPointCloud->numberOfElements;

			// 시각화 범위 설정 (평균 + Sigma * 표준편차)
			// 분산은 항상 양수이므로 minVal은 0으로 고정하는 것이 깔끔함
			float minVal = 0.0f;
			float maxVal = varianceMean + (varianceStdDev * visualizationSigma);
			float range = maxVal - minVal;
			if (range < 1e-6f) range = 1.0f;

			for (size_t i = 0; i < count; ++i)
			{
				float val = normalVariances[i];
				float t = std::clamp((val - minVal) / range, 0.0f, 1.0f);
				if (t > 0.2f)
				{
					marks[i] = 1;
				}
				else
				{
					marks[i] = 0;
				}
			}
		}
		TE(NormalVariance);
	}

	void OperatorNormalVariance::Visualize()
	{
		if (nullptr == cachedPointCloud || normalVariances.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		// 시각화 범위 설정 (평균 + Sigma * 표준편차)
		// 분산은 항상 양수이므로 minVal은 0으로 고정하는 것이 깔끔함
		float minVal = 0.0f;
		float maxVal = varianceMean + (varianceStdDev * visualizationSigma);
		float range = maxVal - minVal;
		if (range < 1e-6f) range = 1.0f;

		for (size_t i = 0; i < count; ++i)
		{
			float val = normalVariances[i];
			float t = std::clamp((val - minVal) / range, 0.0f, 1.0f);

			Eigen::Vector3f color;

			// Blue (Low Variance, Flat) -> Red (High Variance, Edge/Boundary)
			if (t < 0.5f)
			{
				float localT = t / 0.5f;
				color = Eigen::Vector3f(0.0f, localT, 1.0f - localT); // Blue -> Green
			}
			else
			{
				float localT = (t - 0.5f) / 0.5f;
				color = Eigen::Vector3f(localT, 1.0f - localT, 0.0f); // Green -> Red
			}

			// 분산이 높은(경계면) 곳을 강조하기 위해 크기 키움
			float sizeScale = 1.0f + t * 0.5f;


			if (t > 0.2f)
			{
				color = Eigen::Vector3f(1.0f, 0.0f, 0.0f); // Red for high variance
			}
			sizeScale = 1.0f;

			VD::AddSphere(
				"NormalVarianceHeatmap",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius * sizeScale,
				Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
			);
		}
	}
#pragma endregion


#pragma region OperatorNormalDivergence
	OperatorNormalDivergence::OperatorNormalDivergence(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorNormalDivergence::Process()
	{
		TS(NormalDivergence);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		{
			searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
			visualizationScale = parameter.GetParameter<float>("visualizationScale", visualizationScale);
		}

		normalDivergences.assign(numberOfPoints, 0.0f);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				const Eigen::Vector3f& n_i = currentPointCloud->normals[i];

				float divSum = 0.0f;
				float weightSum = 0.0f;

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

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
									Eigen::Vector3f diff = currentPointCloud->positions[curr] - p;
									float distSq = diff.squaredNorm();

									if (distSq <= searchRadiusSq && distSq > 1e-8f)
									{
										float dist = std::sqrt(distSq);
										Eigen::Vector3f dir = diff / dist;

										Eigen::Vector3f n_diff = currentPointCloud->normals[curr] - n_i;

										float dotVal = n_diff.dot(dir);

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

		if (!normalDivergences.empty())
		{
			auto result = std::minmax_element(normalDivergences.begin(), normalDivergences.end());
			minDivergence = *result.first;
			maxDivergence = *result.second;
		}
		else
		{
			minDivergence = 0.0f;
			maxDivergence = 0.0f;
		}

		TE(NormalDivergence);
	}

	void OperatorNormalDivergence::Visualize()
	{
		VisualizeHeatmap();
	}

	void OperatorNormalDivergence::VisualizeHeatmap()
	{
		if (nullptr == cachedPointCloud || normalDivergences.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		double sum = 0.0;
		double sqSum = 0.0;

		for (float v : normalDivergences)
		{
			sum += v;
			sqSum += v * v;
		}

		double mean = sum / count;
		double variance = (sqSum / count) - (mean * mean);
		double stdDev = std::sqrt(std::max(0.0, variance));

		float sigmaMultiplier = 2.0f;

		float limit = (float)(stdDev * sigmaMultiplier);

		if (limit < 1e-6f) limit = 1.0f;

		for (size_t i = 0; i < count; ++i)
		{
			float val = normalDivergences[i];

			float clampedVal = std::clamp(val, -limit, limit);

			float t = (clampedVal + limit) / (2.0f * limit);

			Eigen::Vector3f color;

			if (t < 0.25f)
			{
				float localT = t / 0.25f;
				color = Eigen::Vector3f(0.0f, localT, 1.0f);
			}
			else if (t < 0.5f)
			{
				float localT = (t - 0.25f) / 0.25f;
				color = Eigen::Vector3f(0.0f, 1.0f, 1.0f - localT);
			}
			else if (t < 0.75f)
			{
				float localT = (t - 0.5f) / 0.25f;
				color = Eigen::Vector3f(localT, 1.0f, 0.0f);
			}
			else
			{
				float localT = (t - 0.75f) / 0.25f;
				color = Eigen::Vector3f(1.0f, 1.0f - localT, 0.0f);
			}

			VD::AddSphere(
				"NormalDivergenceHeatmap",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
			);
		}
	}
#pragma endregion
	
#pragma region OperatorNormalDivergenceGradient
	OperatorNormalDivergenceGradient::OperatorNormalDivergenceGradient(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorNormalDivergenceGradient::Process()
	{
		TS(NormalDivergenceGradient);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		{
			searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
			visualizationSigma = parameter.GetParameter<float>("visualizationSigma", visualizationSigma);
		}

		normalDivergences.assign(numberOfPoints, 0.0f);
		divergenceGradients.assign(numberOfPoints, 0.0f);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				const Eigen::Vector3f& n_i = currentPointCloud->normals[i];

				float divSum = 0.0f;
				float weightSum = 0.0f;

				auto ProcessNeighbors = [&](int neighborIdx)
					{
						Eigen::Vector3f diff = currentPointCloud->positions[neighborIdx] - p;
						float distSq = diff.squaredNorm();

						if (distSq <= searchRadiusSq && distSq > 1e-8f)
						{
							float dist = std::sqrt(distSq);
							Eigen::Vector3f dir = diff / dist;
							Eigen::Vector3f n_diff = currentPointCloud->normals[neighborIdx] - n_i;

							float dotVal = n_diff.dot(dir);
							float weight = 1.0f / dist;

							divSum += dotVal * weight;
							weightSum += weight;
						}
					};

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

				for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz) {
					for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy) {
						for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx) {
							uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
							auto it = spatialPartitioning->voxelPointListHead.find(key);
							if (it == spatialPartitioning->voxelPointListHead.end()) continue;
							int curr = it->second;
							while (curr != -1) {
								if (curr != i) ProcessNeighbors(curr);
								curr = spatialPartitioning->nextPoint[curr];
							}
						}
					}
				}

				if (weightSum > 1e-6f) normalDivergences[i] = divSum / weightSum;
			});

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				float myDiv = normalDivergences[i];

				float diffSum = 0.0f;
				int count = 0;

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

				for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz) {
					for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy) {
						for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx) {
							uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
							auto it = spatialPartitioning->voxelPointListHead.find(key);
							if (it == spatialPartitioning->voxelPointListHead.end()) continue;
							int curr = it->second;
							while (curr != -1) {
								if (curr != i) {
									if ((p - currentPointCloud->positions[curr]).squaredNorm() <= searchRadiusSq) {
										diffSum += std::abs(myDiv - normalDivergences[curr]);
										count++;
									}
								}
								curr = spatialPartitioning->nextPoint[curr];
							}
						}
					}
				}

				if (count > 0)
				{
					divergenceGradients[i] = diffSum / (float)count;
				}
			});

		double sum = 0.0;
		double sqSum = 0.0;
		for (float v : divergenceGradients)
		{
			sum += v;
			sqSum += v * v;
		}
		gradientMean = (float)(sum / numberOfPoints);
		float variance = (float)((sqSum / numberOfPoints) - (gradientMean * gradientMean));
		gradientStdDev = std::sqrt(std::max(0.0f, variance));

		TE(NormalDivergenceGradient);
	}

	void OperatorNormalDivergenceGradient::Visualize()
	{
		if (nullptr == cachedPointCloud || divergenceGradients.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		float thresholdLow = gradientMean + gradientStdDev * 0.5f;
		float thresholdHigh = gradientMean + gradientStdDev * visualizationSigma;

		float range = thresholdHigh - thresholdLow;
		if (range < 1e-6f) range = 1.0f;

		for (size_t i = 0; i < count; ++i)
		{
			float val = divergenceGradients[i];

			if (val < thresholdLow)
			{
				VD::AddSphere(
					"Gradient_Stable",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius * 0.8f,
					Eigen::Vector4f(0.5f, 0.5f, 0.5f, 0.1f)
				);
				continue;
			}

			float t = std::clamp((val - thresholdLow) / range, 0.0f, 1.0f);

			Eigen::Vector3f color;

			if (t < 0.25f)
			{
				float localT = t / 0.25f;
				color = Eigen::Vector3f(0.0f, localT, 1.0f);
			}
			else if (t < 0.5f)
			{
				float localT = (t - 0.25f) / 0.25f;
				color = Eigen::Vector3f(0.0f, 1.0f, 1.0f - localT);
			}
			else if (t < 0.75f)
			{
				float localT = (t - 0.5f) / 0.25f;
				color = Eigen::Vector3f(localT, 1.0f, 0.0f);
			}
			else
			{
				float localT = (t - 0.75f) / 0.25f;
				color = Eigen::Vector3f(1.0f, 1.0f - localT, 0.0f);
			}

			float radiusScale = 1.0f + t * 0.5f;

			VD::AddSphere(
				"Gradient_RapidChange",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius * radiusScale,
				Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
			);
		}
	}
#pragma endregion
#pragma endregion

#pragma region OperatorFilterETC
	OperatorFilterETC::OperatorFilterETC(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorFilterETC::Process()
	{
		TS(FilterETC);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		size_t writeIdx = 0;

		bool hasClassIDs = !currentPointCloud->pointDeepLearningClassIDs.empty();
		bool hasClusterIDs = !currentPointCloud->pointClusterIDs.empty();
		bool hasMarks = !currentPointCloud->marks.empty();

		for (size_t readIdx = 0; readIdx < currentPointCloud->numberOfElements; ++readIdx)
		{
			int classID = hasClassIDs ? currentPointCloud->pointDeepLearningClassIDs[readIdx] : -1;

			if (DL_ETC == classID)
			{
				continue;
			}

			if (writeIdx != readIdx)
			{
				currentPointCloud->positions[writeIdx] = currentPointCloud->positions[readIdx];
				currentPointCloud->normals[writeIdx] = currentPointCloud->normals[readIdx];
				currentPointCloud->colors[writeIdx] = currentPointCloud->colors[readIdx];

				if (hasClassIDs)
					currentPointCloud->pointDeepLearningClassIDs[writeIdx] = currentPointCloud->pointDeepLearningClassIDs[readIdx];

				if (hasClusterIDs)
					currentPointCloud->pointClusterIDs[writeIdx] = currentPointCloud->pointClusterIDs[readIdx];

				if (hasMarks)
					currentPointCloud->marks["OperatorFilterETC"][writeIdx] = currentPointCloud->marks["OperatorFilterETC"][readIdx];
			}
			writeIdx++;
		}

		currentPointCloud->numberOfElements = writeIdx;
		currentPointCloud->positions.resize(writeIdx);
		currentPointCloud->normals.resize(writeIdx);
		currentPointCloud->colors.resize(writeIdx);

		if (hasClassIDs) currentPointCloud->pointDeepLearningClassIDs.resize(writeIdx);
		if (hasClusterIDs) currentPointCloud->pointClusterIDs.resize(writeIdx);
		if (hasMarks) currentPointCloud->marks["OperatorFilterETC"].resize(writeIdx);

		cachedPointCloud = currentPointCloud;

		this->parameter.needToRebuildSpatialPartitioning = true;

		TE(FilterETC);
	}

	void OperatorFilterETC::Visualize()
	{
		auto numberOfPoints = cachedPointCloud->numberOfElements;

		for (size_t i = 0; i < numberOfPoints; i++)
		{
			auto& p = cachedPointCloud->positions[i];
			auto& n = cachedPointCloud->normals[i].normalized();
			auto& c = cachedPointCloud->colors[i];
			VD::AddSphere("FilteredPoints", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f));
		}
	}
#pragma endregion

#pragma region OperatorLocalPlaneFitting
	OperatorLocalPlaneFitting::OperatorLocalPlaneFitting(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorLocalPlaneFitting::Process()
	{
		TS(LocalPlaneFitting);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		// 파라미터 로드
		{
			searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
			visualizationScale = parameter.GetParameter<float>("visualizationScale", visualizationScale);
			updatePositions = parameter.GetParameter<bool>("updatePositions", updatePositions);
			updateThreshold = parameter.GetParameter<float>("updateThreshold", updateThreshold); // 0.0 ~ 1.0 사이 값 권장
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
			markedPointsOnly = parameter.GetParameter<bool>("markedPointsOnly", markedPointsOnly);
		}

		fittingResiduals.assign(numberOfPoints, 0.0f);
		fittedPoints.resize(numberOfPoints); // 결과 벡터 크기 할당

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		auto& marks = currentPointCloud->marks["OperatorLocalPlaneFitting"];
		marks.resize(numberOfPoints, 0);

		// [Step 1] 병렬 처리: 피팅 결과 계산 (원본 위치는 건드리지 않고 fittedPoints에만 저장)
		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];

				// [수정] markedPointsOnly 옵션이 켜져있을 경우 마킹되지 않은 점은 연산 제외
				if (markedPointsOnly && marks[i] != 1)
				{
					fittingResiduals[i] = 0.0f;
					fittedPoints[i] = p; // 위치 변경 없음
					return; // 람다 함수 종료 (다음 점으로)
				}

				// 1. 이웃 탐색
				std::vector<int> neighbors;
				neighbors.reserve(64);
				Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

				for (int dz = -neighborSearchOffset; dz <= neighborSearchOffset; ++dz) {
					for (int dy = -neighborSearchOffset; dy <= neighborSearchOffset; ++dy) {
						for (int dx = -neighborSearchOffset; dx <= neighborSearchOffset; ++dx) {
							uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
							auto it = spatialPartitioning->voxelPointListHead.find(key);
							if (it == spatialPartitioning->voxelPointListHead.end()) continue;

							int curr = it->second;
							while (curr != -1) {
								if (curr != i) {
									if ((p - currentPointCloud->positions[curr]).squaredNorm() <= searchRadiusSq) {
										neighbors.push_back(curr);
										centroid += currentPointCloud->positions[curr];
									}
								}
								curr = spatialPartitioning->nextPoint[curr];
							}
						}
					}
				}

				size_t k = neighbors.size();

				// 이웃이 부족하면 피팅 불가 -> 원본 위치 유지, 잔차 0
				if (k < 4)
				{
					fittingResiduals[i] = 0.0f;
					fittedPoints[i] = p;
					return;
				}

				// 2. 공분산 행렬 계산
				centroid /= (float)k;

				float xx = 0.0f, xy = 0.0f, xz = 0.0f;
				float yy = 0.0f, yz = 0.0f, zz = 0.0f;

				for (int idx : neighbors)
				{
					Eigen::Vector3f r = currentPointCloud->positions[idx] - centroid;
					xx += r.x() * r.x(); xy += r.x() * r.y(); xz += r.x() * r.z();
					yy += r.y() * r.y(); yz += r.y() * r.z(); zz += r.z() * r.z();
				}

				Eigen::Matrix3f cov;
				cov(0, 0) = xx; cov(0, 1) = xy; cov(0, 2) = xz;
				cov(1, 0) = xy; cov(1, 1) = yy; cov(1, 2) = yz;
				cov(2, 0) = xz; cov(2, 1) = yz; cov(2, 2) = zz;
				cov /= (float)k;

				// 3. 고유값 분해 (Eigen Decomposition)
				Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);

				float lambda_min = solver.eigenvalues()[0];
				float sum_lambda = solver.eigenvalues().sum();
				Eigen::Vector3f planeNormal = solver.eigenvectors().col(0);

				// 4. Residual 계산 (Surface Variation)
				if (sum_lambda > 1e-9f)
				{
					fittingResiduals[i] = lambda_min / sum_lambda;
				}
				else
				{
					fittingResiduals[i] = 0.0f;
				}

				// 5. 점 투영 (Projection to Plane) - fittedPoints에 임시 저장
				Eigen::Vector3f diff = p - centroid;
				float distToPlane = diff.dot(planeNormal);

				fittedPoints[i] = p - planeNormal * distToPlane;
			});

		// [Step 2] 통계 계산 (Heatmap 및 정규화를 위함)
		double sum = 0.0;
		double sqSum = 0.0;

		// (선택 사항) markedPointsOnly일 때 통계도 마킹된 점만 계산할지 여부는
		// 현재 전체 점 기준으로 계산하여, 마킹 안 된 점들의 0값이 평균을 낮추도록 둠.
		for (float v : fittingResiduals)
		{
			sum += v;
			sqSum += v * v;
		}
		residualMean = (float)(sum / numberOfPoints);
		float variance = (float)((sqSum / numberOfPoints) - (residualMean * residualMean));
		residualStdDev = std::sqrt(std::max(0.0f, variance));

		// [Step 3] 조건부 위치 업데이트
		// updatePositions가 켜져 있을 때만 수행
		if (updatePositions)
		{
			// 정규화 범위 설정 (평균 + 표준편차 * Scale)
			float maxVal = residualMean + residualStdDev * visualizationScale;
			float minVal = 0.0f;
			float range = maxVal - minVal;
			if (range < 1e-6f) range = 1.0f;

			int updatedCount = 0;

			for (size_t i = 0; i < numberOfPoints; ++i)
			{
				// markedPointsOnly로 건너뛴 점들은 잔차(val)가 0이므로 업데이트 대상에서 자연스럽게 제외됨
				float val = fittingResiduals[i];

				// t: 0.0(평면) ~ 1.0(거침/곡률높음) 사이의 정규화된 값
				float t = std::clamp((val - minVal) / range, 0.0f, 1.0f);

				// [핵심] Threshold 이상인 점(거친 부분)만 fittedPoints로 이동
				if (t >= updateThreshold)
				{
					currentPointCloud->positions[i] = fittedPoints[i];
					updatedCount++;
				}
			}

			if (updatedCount > 0)
			{
				// 위치가 변경되었으므로 파티셔닝 재빌드 요청
				parameter.needToRebuildSpatialPartitioning = true;
			}

			printf("Local Plane Fitting: Updated %d points based on fitting residuals.\n", updatedCount);
		}

		TE(LocalPlaneFitting);
	}

	void OperatorLocalPlaneFitting::Visualize()
	{
		if (nullptr == cachedPointCloud || fittingResiduals.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		// Heatmap Range 설정
		// 평균 + (표준편차 * scale)을 최대값으로 설정하여 노이즈에 강건하게 시각화
		float maxVal = residualMean + residualStdDev * visualizationScale;
		float minVal = 0.0f;
		float range = maxVal - minVal;
		if (range < 1e-6f) range = 1.0f;

		
		for (size_t i = 0; i < count; ++i)
		{
			float val = fittingResiduals[i];
			float t = std::clamp((val - minVal) / range, 0.0f, 1.0f);

			Eigen::Vector3f color;

			// Heatmap Color Ramp (Blue -> Cyan -> Green -> Yellow -> Red)
			if (t < 0.25f)
			{
				float localT = t / 0.25f;
				color = Eigen::Vector3f(0.0f, localT, 1.0f); // Blue -> Cyan
			}
			else if (t < 0.5f)
			{
				float localT = (t - 0.25f) / 0.25f;
				color = Eigen::Vector3f(0.0f, 1.0f, 1.0f - localT); // Cyan -> Green
			}
			else if (t < 0.75f)
			{
				float localT = (t - 0.5f) / 0.25f;
				color = Eigen::Vector3f(localT, 1.0f, 0.0f); // Green -> Yellow
			}
			else
			{
				float localT = (t - 0.75f) / 0.25f;
				color = Eigen::Vector3f(1.0f, 1.0f - localT, 0.0f); // Yellow -> Red
			}

			// 평면에 잘 맞을수록(Blue) 작게, 안 맞을수록(Red) 크게 표시
			float sizeScale = 1.0f + t * 0.5f;

			VD::AddSphere(
				"PlaneFittingHeatmap",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius * sizeScale,
				Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
			);
		}
		

		//for (size_t i = 0; i < count; ++i)
		//{
		//	float val = fittingResiduals[i];
		//	float t = std::clamp((val - minVal) / range, 0.0f, 1.0f);

		//	Eigen::Vector3f color = cachedPointCloud->colors[i];

		//	if (t > 0.75f)
		//	{
		//		float localT = (t - 0.75f) / 0.25f;
		//		color = Eigen::Vector3f(1.0f, 1.0f - localT, 0.0f); // Yellow -> Red
		//	}

		//	// 평면에 잘 맞을수록(Blue) 작게, 안 맞을수록(Red) 크게 표시
		//	float sizeScale = 1.0f + t * 0.5f;

		//	VD::AddSphere(
		//		"PlaneFittingHeatmap",
		//		cachedPointCloud->positions[i],
		//		Configuration::pointVisualizationRadius * sizeScale,
		//		Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
		//	);
		//}
	}
#pragma endregion

#pragma region OperatorCompareSameIndexOrderedPointCloud
	OperatorCompareSameIndexOrderedPointCloud::OperatorCompareSameIndexOrderedPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorCompareSameIndexOrderedPointCloud::Process()
	{
		TS(CompareOrderedPointCloud);

		int indexA = parameter.GetParameter<int>("indexA", 0);
		int indexB = parameter.GetParameter<int>("indexB", -1);

		pointCloudA = pipeline->GetPointCloud(indexA);
		pointCloudB = pipeline->GetPointCloud(indexB);

		if (!pointCloudA || !pointCloudB)
		{
			TE(CompareOrderedPointCloud);
			return;
		}

		size_t countA = pointCloudA->numberOfElements;
		size_t countB = pointCloudB->numberOfElements;

		matchedFlags.assign(countA, 0);
		deletedCount = 0;

		size_t idxB = 0;
		for (size_t idxA = 0; idxA < countA; ++idxA)
		{
			bool isMatch = false;

			if (idxB < countB)
			{
				const Eigen::Vector3f& pA = pointCloudA->positions[idxA];
				const Eigen::Vector3f& pB = pointCloudB->positions[idxB];

				if ((pA - pB).squaredNorm() <= comparisonDistanceThresholdSq)
				{
					isMatch = true;
				}
			}

			if (isMatch)
			{
				matchedFlags[idxA] = 1;
				idxB++;
			}
			else
			{
				matchedFlags[idxA] = 0;
				deletedCount++;
			}
		}

		alog("Ordered Compare Result: Total A(%zu), Total B(%zu), Deleted(%d)\n", countA, countB, deletedCount);

		cachedPointCloud = pointCloudA;
		TE(CompareOrderedPointCloud);
	}

	void OperatorCompareSameIndexOrderedPointCloud::Visualize()
	{
		if (!pointCloudA || matchedFlags.empty()) return;

		for (size_t i = 0; i < pointCloudA->numberOfElements; ++i)
		{
			const auto& p = pointCloudA->positions[i];
			const auto& n = pointCloudA->normals[i].normalized();

			if (matchedFlags[i])
			{
				VD::AddSphere(
					"OrderedCompare_Matched",
					p, n,
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(0.0f, 1.0f, 0.0f, 0.2f));
			}
			else
			{
				VD::AddSphere(
					"OrderedCompare_Deleted",
					p, n,
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
			}
		}
	}
#pragma endregion

#pragma region OperatorComparePointCloudUsingDistance
	OperatorComparePointCloudUsingDistance::OperatorComparePointCloudUsingDistance(Pipeline* pipeline,
		const GeometricProcessingOperatorParameter& parameter) : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorComparePointCloudUsingDistance::Process()
	{
		TS(ComparePointCloudUsingDistance);

		int indexA = parameter.GetParameter<int>("indexA", 0);
		int indexB = parameter.GetParameter<int>("indexB", -1);

		pointCloudA = pipeline->GetPointCloud(indexA);
		pointCloudB = pipeline->GetPointCloud(indexB);

		if (!pointCloudA || !pointCloudB ||
			pointCloudA->numberOfElements == 0 ||
			pointCloudB->numberOfElements == 0)
		{
			TE(ComparePointCloudUsingDistance);
			return;
		}

		SparseGrid gridB;
		gridB.Build(*pointCloudB, Configuration::voxelSize);

		size_t countA = pointCloudA->numberOfElements;
		matchedFlags.assign(countA, 0);

		float thresholdSq = comparisonDistanceThreshold * comparisonDistanceThreshold;

		std::vector<int> indices(countA);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(),
			[&](int i)
			{
				const Eigen::Vector3f& pA = pointCloudA->positions[i];

				int gx = (int)std::floor((pA.x() - gridB.aabb.min.x()) / gridB.cellSize);
				int gy = (int)std::floor((pA.y() - gridB.aabb.min.y()) / gridB.cellSize);
				int gz = (int)std::floor((pA.z() - gridB.aabb.min.z()) / gridB.cellSize);

				bool matched = false;

				for (int dz = -1; dz <= 1 && !matched; ++dz)
				{
					for (int dy = -1; dy <= 1 && !matched; ++dy)
					{
						for (int dx = -1; dx <= 1 && !matched; ++dx)
						{
							uint64_t key = gridB.GetKey(gx + dx, gy + dy, gz + dz);
							auto it = gridB.voxelPointListHead.find(key);
							if (it == gridB.voxelPointListHead.end())
								continue;

							int curr = it->second;
							while (curr != -1)
							{
								if ((pA - pointCloudB->positions[curr]).squaredNorm() <= thresholdSq)
								{
									matched = true;
									break;
								}
								curr = gridB.nextPoint[curr];
							}
						}
					}
				}

				matchedFlags[i] = matched ? 1 : 0;
			});

		cachedPointCloud = pointCloudA;
		TE(ComparePointCloudUsingDistance);
	}

	void OperatorComparePointCloudUsingDistance::Visualize()
	{
		if (!pointCloudA || matchedFlags.empty())
			return;

		for (size_t i = 0; i < pointCloudA->numberOfElements; ++i)
		{
			const auto& p = pointCloudA->positions[i];
			const auto& n = pointCloudA->normals[i].normalized();

			if (matchedFlags[i])
			{
				VD::AddSphere(
					"Compare_Matched",
					p, n,
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(0.0f, 1.0f, 0.0f, 1.0f));
			}
			else
			{
				VD::AddSphere(
					"Compare_Unmatched",
					p, n,
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
			}
		}
	}
#pragma endregion

#pragma region OperatorCompareWithLastPointCloud
	OperatorCompareWithLastPointCloud::OperatorCompareWithLastPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorCompareWithLastPointCloud::Process()
	{
		TS(CompareWithLastPointCloud);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();

		if (currentPointCloud->numberOfElements == 0) return;
		if (nullptr == pipeline->GetLastPointCloud()) return;
		cachedPointCloud = currentPointCloud;

		auto cp = currentPointCloud;
		auto lp = pipeline->GetLastPointCloud();

		size_t numberOfPoints = cp->numberOfElements > lp->numberOfElements ? cp->numberOfElements : lp->numberOfElements;
		matchedFlags.resize(numberOfPoints, false);
		for (size_t i = 0; i < numberOfPoints; i++)
		{
			if (i < cp->numberOfElements && i < lp->numberOfElements)
			{
				if (comparisonDistanceThreshold < (cp->positions[i] - lp->positions[i]).norm())
				{
					matchedFlags[i] = true;
				}
				else
				{
					matchedFlags[i] = false;
				}
			}
			else
			{
				matchedFlags[i] = false;
			}
		}

		TE(CompareWithLastPointCloud);
	}

	void OperatorCompareWithLastPointCloud::Visualize()
	{
		if (nullptr == cachedPointCloud) return;
		if (cachedPointCloud->numberOfElements == 0) return;
		if (nullptr == pipeline->GetLastPointCloud()) return;

		auto cp = cachedPointCloud;
		auto lp = pipeline->GetLastPointCloud();

		size_t numberOfPoints = cp->numberOfElements > lp->numberOfElements ? cp->numberOfElements : lp->numberOfElements;
		auto pc = cp->numberOfElements > lp->numberOfElements ? cp : lp;
		for (size_t i = 0; i < numberOfPoints; ++i)
		{
			auto& p = cp->positions[i];
			auto& n = cp->normals[i].normalized();
			auto& c = cp->colors[i];

			if (cp->numberOfElements > lp->numberOfElements)
			{
				if (matchedFlags[i])
				{
					VD::AddSphere("ComparisonResult_Matched", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f));
				}
				else
				{
					VD::AddSphere("ComparisonResult_Unmatched", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
				}
			}
			else
			{
				if (i < cp->numberOfElements)
				{
					if (matchedFlags[i])
					{
						VD::AddSphere("ComparisonResult_Matched", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f));
					}
					else
					{
						VD::AddSphere("ComparisonResult_Unmatched", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));
					}
				}
				else
				{
					auto& p2 = lp->positions[i];
					auto& n2 = lp->normals[i].normalized();
					auto& c2 = lp->colors[i];
					VD::AddSphere("ComparisonResult_Unmatched", p2, n2, Configuration::pointVisualizationRadius, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f));
				}
			}
		}
	}
#pragma endregion

#pragma region OperatorClustering
	OperatorClustering::OperatorClustering(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorClustering::Process()
	{
		TS(Clustering_Parallel);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();

		if (currentPointCloud->numberOfElements == 0) return;
		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		if (numberOfPoints != currentPointCloud->pointClusterIDs.size())
		{
			currentPointCloud->pointClusterIDs.resize(numberOfPoints, -1);
		}
		else
		{
			std::fill(currentPointCloud->pointClusterIDs.begin(), currentPointCloud->pointClusterIDs.end(), -1);
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

		auto& marks = currentPointCloud->marks["OperatorClustering"];
		marks.resize(numberOfPoints, 0);

		std::for_each(std::execution::par, flatCells.begin(), flatCells.end(), [&](const CellData& cell)
			{
				uint64_t key = cell.key;
				int headIdx = cell.headIdx;

				int gz = (int)(key & mask);
				int gy = (int)((key >> 21) & mask);
				int gx = (int)(key >> 42);

				for (int i = headIdx; i != -1; i = spatialPartitioning->nextPoint[i])
				{
					const Eigen::Vector3f& pA = currentPointCloud->positions[i];
					const Eigen::Vector3f& nA = currentPointCloud->normals[i];

					for (int j = spatialPartitioning->nextPoint[i]; j != -1; j = spatialPartitioning->nextPoint[j])
					{
						const Eigen::Vector3f& pB = currentPointCloud->positions[j];

						if ((pA - pB).squaredNorm() > searchRadiusSq) continue;

						const Eigen::Vector3f& nB = currentPointCloud->normals[j];

						if (nA.dot(nB) < strictAngleThreshold) continue;

						float planeDist = std::abs(nA.dot(pB - pA));
						if (planeDist > planeDistThreshold) continue;

						if (useMarksForClustering)
						{
							if (marks[i] != marks[j]) continue;
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
								if (neighborKey < key) continue;

								auto it = spatialPartitioning->voxelPointListHead.find(neighborKey);
								if (it == spatialPartitioning->voxelPointListHead.end()) continue;

								int neighborHead = it->second;
								for (int j = neighborHead; j != -1; j = spatialPartitioning->nextPoint[j])
								{
									const Eigen::Vector3f& pB = currentPointCloud->positions[j];

									if ((pA - pB).squaredNorm() > searchRadiusSq) continue;

									const Eigen::Vector3f& nB = currentPointCloud->normals[j];

									if (nA.dot(nB) < strictAngleThreshold) continue;

									float planeDist = std::abs(nA.dot(pB - pA));
									if (planeDist > planeDistThreshold) continue;

									if (useMarksForClustering)
									{
										if (marks[i] != marks[j]) continue;
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
			currentPointCloud->pointClusterIDs[i] = rootToClusterID[root];
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
				currentPointCloud->pointClusterIDs[i] = rootToSortedId[root];
			}

			alog("Clustering Done. Found %d clusters.\n", currentClusterCount);
			if (sortedClusters.size() > 0) alog(" - Biggest(ID 0): %d points\n", sortedClusters[0].second);
			if (sortedClusters.size() > 1) alog(" - 2nd(ID 1): %d points\n", sortedClusters[1].second);
			if (sortedClusters.size() > 2) alog(" - 3rd(ID 2): %d points\n", sortedClusters[2].second);
		}

		TE(Clustering_Parallel);
	}

	void OperatorClustering::Visualize()
	{
		if (nullptr == cachedPointCloud || cachedPointCloud->pointClusterIDs.empty()) return;

		auto contrastingColors = Color::GetContrastingColorsWithoutBWRGB(256);

		size_t count = cachedPointCloud->numberOfElements;
		for (size_t i = 0; i < count; ++i)
		{
			int clusterId = cachedPointCloud->pointClusterIDs[i];
			if (clusterId < 0) continue;

			const Eigen::Vector3f& p = cachedPointCloud->positions[i];

			auto color = contrastingColors[clusterId % contrastingColors.size()];

			VD::AddSphere(
				"ClusteredPoints",
				p,
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(color.r, color.g, color.b, color.a)
			);
		}
	}
#pragma endregion

#pragma region OperatorClusterBorderFinding
	OperatorClusterBorderFinding::OperatorClusterBorderFinding(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorClusterBorderFinding::Process()
	{
		TS(ClusterBorderFinding);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;
		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;
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
					const Eigen::Vector3f& pA = currentPointCloud->positions[i];
					int clusterA = currentPointCloud->pointClusterIDs[i];
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
									const Eigen::Vector3f& pB = currentPointCloud->positions[j];
									if ((pA - pB).squaredNorm() > searchRadiusSq) continue;
									int clusterB = currentPointCloud->pointClusterIDs[j];
									if (0 != clusterA && 0 == clusterB)
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

	void OperatorClusterBorderFinding::Visualize()
	{
		if (nullptr == cachedPointCloud) return;

		if (cachedPointCloud->marks.find("OperatorClustering") == cachedPointCloud->marks.end())
		{
			cachedPointCloud->marks["OperatorClustering"] = std::vector<int>(cachedPointCloud->numberOfElements, 0);
		}

		for (const auto& idx : borderPointIndices)
		{
			cachedPointCloud->marks["OperatorClustering"][idx] = 1;
		}

		auto& marks = cachedPointCloud->marks["OperatorClusterBorderFinding"];

		for (size_t i = 0; i < cachedPointCloud->numberOfElements; i++)
		{
			auto& p = cachedPointCloud->positions[i];
			auto& n = cachedPointCloud->normals[i].normalized();
			auto& c = cachedPointCloud->colors[i];
			auto& mark = marks[i];

			if (1 == mark)
			{
				VD::AddSphere(
					"BorderPoints_Mark1",
					p,
					n,
					Configuration::pointVisualizationRadius * 1.2f,
					Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f)
				);
			}
			else
			{
				VD::AddSphere("AllPoints", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f));
			}
		}
	}
#pragma endregion

#pragma region OperatorClusteringComplex
	OperatorClusteringComplex::OperatorClusteringComplex(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorClusteringComplex::Process()
	{
		TS(ComplexClustering);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;
		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;
		pointClusterIds.assign(numberOfPoints, -1);
		pointCurvatures.resize(numberOfPoints);

		bool hasValidClassIDs = params.useDeepLearningClasses &&
			!currentPointCloud->pointDeepLearningClassIDs.empty() &&
			(currentPointCloud->pointDeepLearningClassIDs.size() == numberOfPoints);

		TS(PrecalcFeatures);
		{
			std::vector<int> indices(numberOfPoints);
			std::iota(indices.begin(), indices.end(), 0);
			float curvSearchRadSq = std::pow(spatialPartitioning->cellSize * 2.0f, 2.0f);

			std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
				{
					Eigen::Vector3f center = currentPointCloud->positions[i];
					int neighbors = 0;

					int gx = (int)std::floor((center.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
					int gy = (int)std::floor((center.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
					int gz = (int)std::floor((center.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

					float sumDistSq = 0.0f;

					for (int dz = -1; dz <= 1; ++dz) {
						for (int dy = -1; dy <= 1; ++dy) {
							for (int dx = -1; dx <= 1; ++dx) {
								uint64_t key = spatialPartitioning->GetKey(gx + dx, gy + dy, gz + dz);
								auto it = spatialPartitioning->voxelPointListHead.find(key);
								if (it == spatialPartitioning->voxelPointListHead.end()) continue;

								for (int idx = it->second; idx != -1; idx = spatialPartitioning->nextPoint[idx]) {
									if (i == idx) continue;
									if ((center - currentPointCloud->positions[idx]).squaredNorm() <= curvSearchRadSq) {
										float dot = currentPointCloud->normals[i].dot(currentPointCloud->normals[idx]);
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

		TS(ClusteringExecution);

		AtomicDisjointSet dsu;
		dsu.Initialize(numberOfPoints);

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
					const Eigen::Vector3f& pA = currentPointCloud->positions[i];
					const Eigen::Vector3f& nA = currentPointCloud->normals[i];
					const Eigen::Vector3f& cA = currentPointCloud->colors[i];
					float curvA = pointCurvatures[i];
					int classA = hasValidClassIDs ? currentPointCloud->pointDeepLearningClassIDs[i] : -1;

					auto CheckAndMerge = [&](int j)
						{
							if (hasValidClassIDs)
							{
								if (classA != currentPointCloud->pointDeepLearningClassIDs[j]) return;
							}

							const Eigen::Vector3f& pB = currentPointCloud->positions[j];

							if ((pA - pB).squaredNorm() > searchRadiusSq) return;

							const Eigen::Vector3f& nB = currentPointCloud->normals[j];

							if (nA.dot(nB) < params.angleThreshold) return;

							if (std::abs(nA.dot(pB - pA)) > planeDistAbs) return;

							if ((cA - currentPointCloud->colors[j]).norm() > params.colorThreshold) return;

							if (std::abs(curvA - pointCurvatures[j]) > params.curvatureDiffThreshold) return;

							dsu.Union(i, j);
						};

					for (int j = spatialPartitioning->nextPoint[i]; j != -1; j = spatialPartitioning->nextPoint[j]) {
						CheckAndMerge(j);
					}

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

		std::map<int, int> rootToId;
		int clusterCount = 0;
		for (size_t i = 0; i < numberOfPoints; ++i)
		{
			int root = dsu.Find((int)i);
			if (rootToId.find(root) == rootToId.end()) rootToId[root] = clusterCount++;
			pointClusterIds[i] = rootToId[root];
		}

		alog("Complex Clustering Done. Found %d clusters.\n", clusterCount);
		TE(ClusteringExecution);
		TE(ComplexClustering);
	}

	void OperatorClusteringComplex::Visualize()
	{
		if (nullptr == cachedPointCloud || pointClusterIds.empty()) return;
		auto colors = Color::GetContrastingColors(64);

		for (size_t i = 0; i < cachedPointCloud->numberOfElements; ++i)
		{
			int id = pointClusterIds[i];
			if (id < 0) continue;

			auto color = colors[id % 64];

			VD::AddSphere("ComplexClusters",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				Eigen::Vector4f(color.r, color.g, color.b, color.a));
		}
	}
#pragma endregion

#pragma region OperatorCurvatureEstimation
	OperatorCurvatureEstimation::OperatorCurvatureEstimation(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorCurvatureEstimation::Process()
	{
		TS(Curvature_Parallel);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		{ // Apply parameters
			curvatureThreshold = parameter.GetParameter<float>("curvatureThreshold", curvatureThreshold);
			neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
			searchRadiusScale = parameter.GetParameter<float>("searchRadiusScale", searchRadiusScale);
			visualScale = parameter.GetParameter<float>("visualScale", visualScale);
		}

		curvatures.resize(numberOfPoints);

		currentPointCloud->marks["OperatorCurvatureEstimation"].clear();
		currentPointCloud->marks["OperatorCurvatureEstimation"].resize(numberOfPoints, 0);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusScale;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		auto& marks = currentPointCloud->marks["OperatorCurvatureEstimation"];
		marks.resize(numberOfPoints, 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];

				std::vector<int> neighbors;
				neighbors.reserve(64);

				Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

				int gx = (int)std::floor((p.x() - spatialPartitioning->aabb.min.x()) / spatialPartitioning->cellSize);
				int gy = (int)std::floor((p.y() - spatialPartitioning->aabb.min.y()) / spatialPartitioning->cellSize);
				int gz = (int)std::floor((p.z() - spatialPartitioning->aabb.min.z()) / spatialPartitioning->cellSize);

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
									if ((p - currentPointCloud->positions[currIdx]).squaredNorm() <= searchRadiusSq)
									{
										neighbors.push_back(currIdx);
										centroid += currentPointCloud->positions[currIdx];
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

				float xx = 0, xy = 0, xz = 0;
				float yy = 0, yz = 0, zz = 0;

				for (int idx : neighbors)
				{
					Eigen::Vector3f r = currentPointCloud->positions[idx] - centroid;
					xx += r.x() * r.x();
					xy += r.x() * r.y();
					xz += r.x() * r.z();
					yy += r.y() * r.y();
					yz += r.y() * r.z();
					zz += r.z() * r.z();
				}

				Eigen::Matrix3f cov;
				cov(0, 0) = xx; cov(0, 1) = xy; cov(0, 2) = xz;
				cov(1, 0) = xy; cov(1, 1) = yy; cov(1, 2) = yz;
				cov(2, 0) = xz; cov(2, 1) = yz; cov(2, 2) = zz;

				cov /= (float)k;

				Eigen::Vector3f evals = ComputeEigenValuesSymmetric(cov);

				float l0 = evals.x(), l1 = evals.y(), l2 = evals.z();
				if (l0 > l1) std::swap(l0, l1);
				if (l1 > l2) std::swap(l1, l2);
				if (l0 > l1) std::swap(l0, l1);

				float sum = l0 + l1 + l2;
				if (sum > 1e-9f)
				{
					curvatures[i] = l0 / sum;
				}
				else
				{
					curvatures[i] = 0.0f;
				}

				if (curvatureThreshold < curvatures[i])
				{
					marks[i] = 1;
				}
			});

		TE(Curvature_Parallel);
	}

	void OperatorCurvatureEstimation::Visualize()
	{
		if (nullptr == cachedPointCloud || curvatures.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		auto [curvatureMin, curvatureMax] = std::minmax_element(curvatures.begin(), curvatures.end());

		auto& marks = cachedPointCloud->marks["OperatorCurvatureEstimation"];

		for (size_t i = 0; i < count; ++i)
		{
			float val = curvatures[i];

			Eigen::Vector4f color = Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f);
			if (marks[i] == 1)
			{
				color = Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f);
			}
			else
			{
				color = Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f);
			}

			VD::AddSphere(
				"CurvatureEstimation",
				cachedPointCloud->positions[i],
				Configuration::pointVisualizationRadius,
				color
			);
		}
	}
#pragma endregion

#pragma region OperatorCurvatureDivergence
	OperatorCurvatureDivergence::OperatorCurvatureDivergence(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorCurvatureDivergence::Process()
	{
		TS(CurvatureDivergence_Total);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		curvatures.assign(numberOfPoints, 0.0f);
		gradients.assign(numberOfPoints, Eigen::Vector3f::Zero());
		divergences.assign(numberOfPoints, 0.0f);

		float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
		float searchRadiusSq = searchRadius * searchRadius;

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		TS(Pass1_Curvature);
		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				std::vector<int> neighbors;
				neighbors.reserve(32);
				Eigen::Vector3f centroid = Eigen::Vector3f::Zero();

				ProcessNeighbors(i, p, *spatialPartitioning, searchRadiusSq, [&](int neighborIdx) {
					neighbors.push_back(neighborIdx);
					centroid += currentPointCloud->positions[neighborIdx];
					});

				size_t k = neighbors.size();
				if (k < 4) return;

				centroid /= (float)k;

				float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
				for (int idx : neighbors)
				{
					Eigen::Vector3f r = currentPointCloud->positions[idx] - centroid;
					xx += r.x() * r.x(); xy += r.x() * r.y(); xz += r.x() * r.z();
					yy += r.y() * r.y(); yz += r.y() * r.z(); zz += r.z() * r.z();
				}

				Eigen::Matrix3f cov;
				cov(0, 0) = xx; cov(0, 1) = xy; cov(0, 2) = xz;
				cov(1, 0) = xy; cov(1, 1) = yy; cov(1, 2) = yz;
				cov(2, 0) = xz; cov(2, 1) = yz; cov(2, 2) = zz;
				cov /= (float)k;

				Eigen::Vector3f evals = ComputeEigenValuesSymmetric(cov);
				float l0 = evals.x(), l1 = evals.y(), l2 = evals.z();
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
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				float c_i = curvatures[i];
				Eigen::Vector3f gradSum = Eigen::Vector3f::Zero();
				float weightSum = 0.0f;

				ProcessNeighbors(i, p, *spatialPartitioning, searchRadiusSq, [&](int j) {
					Eigen::Vector3f diff = currentPointCloud->positions[j] - p;
					float dist = diff.norm();
					if (dist > 1e-6f)
					{
						Eigen::Vector3f dir = diff / dist;
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
				const Eigen::Vector3f& p = currentPointCloud->positions[i];
				const Eigen::Vector3f& g_i = gradients[i];
				float divSum = 0.0f;
				float count = 0.0f;

				ProcessNeighbors(i, p, *spatialPartitioning, searchRadiusSq, [&](int j) {
					Eigen::Vector3f diff = currentPointCloud->positions[j] - p;
					float dist = diff.norm();
					if (dist > 1e-6f)
					{
						Eigen::Vector3f dir = diff / dist;
						Eigen::Vector3f g_diff = gradients[j] - g_i;
						divSum += g_diff.dot(dir);
						count += 1.0f;
					}
					});

				if (count > 0.5f) divergences[i] = divSum / count;
			});
		TE(Pass3_Divergence);
		TE(CurvatureDivergence_Total);
	}

	void OperatorCurvatureDivergence::Visualize()
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
					Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f)
				);

				continue;
			}

			float t = std::clamp(val * visualizationScale + 0.5f, 0.0f, 1.0f);

			Eigen::Vector3f color;
			if (t < 0.5f) {
				float localT = t * 2.0f;
				color = Eigen::Vector3f(0, 0, 1) * (1.0f - localT) + Eigen::Vector3f(0.5f, 0.5f, 0.5f) * localT;

				VD::AddSphere(
					"CurvatureDivergence_sink",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
				);
			}
			else {
				float localT = (t - 0.5f) * 2.0f;
				color = Eigen::Vector3f(0.5f, 0.5f, 0.5f) * (1.0f - localT) + Eigen::Vector3f(1, 0, 0) * localT;

				VD::AddSphere(
					"CurvatureDivergence_source",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
				);
			}
		}
	}
#pragma endregion

#pragma region OperatorMeshGeneration
	OperatorMeshGeneration::OperatorMeshGeneration(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorMeshGeneration::Process()
	{
		TS(MeshGeneration);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0) return;

		if (nullptr == spatialPartitioning)
		{
			spatialPartitioning = new SparseGrid();
			spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			parameter.needToDeleteSpatialPartitioning = true;
		}

		cachedPointCloud = currentPointCloud;

		sparseDataBlock.dataBlocks.clear();
		sparseDataBlock.voxelSize = meshVoxelSize;

		sparseDataBlock.FromPointsData(
			currentPointCloud->positions,
			currentPointCloud->normals,
			currentPointCloud->colors,
			currentPointCloud->pointClusterIDs,
			spatialPartitioning->aabb.min
		);

		meshGenerator.Generate(sparseDataBlock);

		if (false == exportFilename.empty())
		{
			meshGenerator.ExportPLY(exportFilename);
		}

		if (detectHoles)
		{
			meshGenerator.DetectHoles();
		}

		TE(MeshGeneration);
	}

	void OperatorMeshGeneration::Visualize()
	{
		meshGenerator.Visualize(showMesh, showHoles);
	}

	void OperatorMeshGeneration::ExportPLY(const std::string& filename)
	{
		exportFilename = filename;
	}

#pragma endregion
	
#pragma region OperatorMeshDistanceFilter
	OperatorMeshDistanceFilter::OperatorMeshDistanceFilter(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
		: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
	{
	}

	void OperatorMeshDistanceFilter::SetReferenceMesh(const std::vector<Triangle>& meshTriangles)
	{
		referenceMesh.clear();
		referenceMesh.reserve(meshTriangles.size());

		for (const auto& tri : meshTriangles)
		{
			Eigen::Vector3f e1 = tri.v[1] - tri.v[0];
			Eigen::Vector3f e2 = tri.v[2] - tri.v[0];
			Eigen::Vector3f crossP = e1.cross(e2);

			if (crossP.squaredNorm() > 1e-12f)
			{
				referenceMesh.push_back(tri);
			}
		}
	}

	void OperatorMeshDistanceFilter::Process()
	{
		TS(MeshDistanceFilter);

		auto currentPointCloud = pipeline->GetCurrentPointCloud();
		if (currentPointCloud->numberOfElements == 0 || referenceMesh.empty()) return;

		cachedPointCloud = currentPointCloud;
		size_t numberOfPoints = currentPointCloud->numberOfElements;

		distances.resize(numberOfPoints);
		currentPointCloud->marks["OperatorMeshDistanceFilter"].assign(numberOfPoints, 0);

		BuildTriangleGrid();

		std::vector<int> indices(numberOfPoints);
		std::iota(indices.begin(), indices.end(), 0);

		std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
			{
				float d = GetClosestDistanceFromMesh(currentPointCloud->positions[i]);

				if (std::isnan(d) || std::isinf(d)) d = FLT_MAX;

				distances[i] = d;
			});

		auto& marks = currentPointCloud->marks["OperatorMeshDistanceFilter"];
		marks.resize(numberOfPoints, 0);

		int markedCount = 0;
		for (size_t i = 0; i < numberOfPoints; ++i)
		{
			if (distances[i] > Configuration::voxelSize * thresholdMultiplier)
			{
				marks[i] = 1;
				markedCount++;
			}
		}

		TE(MeshDistanceFilter);
	}

	void OperatorMeshDistanceFilter::Visualize()
	{
		if (nullptr == cachedPointCloud || distances.empty()) return;

		size_t count = cachedPointCloud->numberOfElements;

		auto& marks = cachedPointCloud->marks["OperatorMeshDistanceFilter"];

		for (size_t i = 0; i < count; ++i)
		{
			if (marks[i] == 1)
			{
				VD::AddSphere(
					"HighDistancePoints",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius * 1.2f,
					Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f)
				);
			}
			else
			{
				float t = (averageDistance > 1e-6f) ? std::clamp(distances[i] / (averageDistance * 2.0f), 0.0f, 1.0f) : 0.0f;
				Eigen::Vector3f color = Eigen::Vector3f(0, 0, 1) * (1.0f - t) + Eigen::Vector3f(0, 1, 1) * t;

				VD::AddSphere(
					"NormalDistancePoints",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(color.x(), color.y(), color.z(), 0.5f)
				);
			}
		}
	}

	void OperatorMeshDistanceFilter::BuildTriangleGrid()
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
		triGridMin = meshAABB.min - Eigen::Vector3f::Constant(0.1f);
		triGridSize = Configuration::voxelSize * 5.0f;

		for (int i = 0; i < (int)referenceMesh.size(); ++i)
		{
			const auto& tri = referenceMesh[i];

			Eigen::Vector3f tMin = tri.v[0].cwiseMin(tri.v[1]).cwiseMin(tri.v[2]);
			Eigen::Vector3f tMax = tri.v[0].cwiseMax(tri.v[1]).cwiseMax(tri.v[2]);

			int minX = (int)std::floor((tMin.x() - triGridMin.x()) / triGridSize);
			int minY = (int)std::floor((tMin.y() - triGridMin.y()) / triGridSize);
			int minZ = (int)std::floor((tMin.z() - triGridMin.z()) / triGridSize);

			int maxX = (int)std::floor((tMax.x() - triGridMin.x()) / triGridSize);
			int maxY = (int)std::floor((tMax.y() - triGridMin.y()) / triGridSize);
			int maxZ = (int)std::floor((tMax.z() - triGridMin.z()) / triGridSize);

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

	float OperatorMeshDistanceFilter::GetClosestDistanceFromMesh(const Eigen::Vector3f& p)
	{
		float minDistSq = FLT_MAX;

		int gx = (int)std::floor((p.x() - triGridMin.x()) / triGridSize);
		int gy = (int)std::floor((p.y() - triGridMin.y()) / triGridSize);
		int gz = (int)std::floor((p.z() - triGridMin.z()) / triGridSize);

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

	float OperatorMeshDistanceFilter::SqDistPointTriangle(const Eigen::Vector3f& p, const Triangle& tri)
	{
		Eigen::Vector3f B = tri.v[0];
		Eigen::Vector3f E0 = tri.v[1] - B;
		Eigen::Vector3f E1 = tri.v[2] - B;
		Eigen::Vector3f D = B - p;
		float a = E0.dot(E0);
		float b = E0.dot(E1);
		float c = E1.dot(E1);
		float d = E0.dot(D);
		float e = E1.dot(D);
		float f = D.dot(D);

		float det = a * c - b * b;
		float s = b * e - c * d;
		float t = b * d - a * e;

		if (std::abs(det) < 1e-12f)
		{
			float d0 = (p - tri.v[0]).squaredNorm();
			float d1 = (p - tri.v[1]).squaredNorm();
			float d2 = (p - tri.v[2]).squaredNorm();
			return std::min({ d0, d1, d2 });
		}

		if (s + t <= det)
		{
			if (s < 0.f)
			{
				if (t < 0.f)
				{
					if (d < 0.f) { t = 0.f; if (-d >= a) { s = 1.f; } else { s = -d / a; } }
					else { s = 0.f; if (e >= 0.f) { t = 0.f; } else if (-e >= c) { t = 1.f; } else { t = -e / c; } }
				}
				else
				{
					s = 0.f; if (e >= 0.f) { t = 0.f; }
					else if (-e >= c) { t = 1.f; }
					else { t = -e / c; }
				}
			}
			else if (t < 0.f)
			{
				t = 0.f; if (d >= 0.f) { s = 0.f; }
				else if (-d >= a) { s = 1.f; }
				else { s = -d / a; }
			}
			else
			{
				float invDet = 1.f / det; s *= invDet; t *= invDet;
			}
		}
		else
		{
			if (s < 0.f)
			{
				float tmp0 = b + d; float tmp1 = c + e;
				if (tmp1 > tmp0) { float numer = tmp1 - tmp0; float denom = a - 2.f * b + c; s = (numer >= denom) ? 1.f : numer / denom; t = 1.f - s; }
				else { s = 0.f; if (tmp1 <= 0.f) { t = 1.f; } else if (e >= 0.f) { t = 0.f; } else { t = -e / c; } }
			}
			else if (t < 0.f)
			{
				float tmp0 = b + e; float tmp1 = a + d;
				if (tmp1 > tmp0) { float numer = tmp1 - tmp0; float denom = a - 2.f * b + c; t = (numer >= denom) ? 1.f : numer / denom; s = 1.f - t; }
				else { t = 0.f; if (tmp1 <= 0.f) { s = 1.f; } else if (d >= 0.f) { s = 0.f; } else { s = -d / a; } }
			}
			else
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
#pragma endregion
}
