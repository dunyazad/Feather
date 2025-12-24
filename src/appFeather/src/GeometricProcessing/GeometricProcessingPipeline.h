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
	case DL_DENTIFORM_TOOTH:
	case DL_METAL:
	case DL_ABUTMENT:
	case DL_SCANBODY:
		return true;
	default:
		return false;
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

		Ray(const Eigen::Vector3f& o, const Eigen::Vector3f& d);

		bool IntersectSphere(const Eigen::Vector3f& sphereCenter, float radius, float& t) const;
	};

	struct AABB
	{
		Eigen::Vector3f min = Eigen::Vector3f::Constant(FLT_MAX);
		Eigen::Vector3f max = Eigen::Vector3f::Constant(-FLT_MAX);

		bool Intersects(const AABB& other) const;
		bool Contains(const Eigen::Vector3f& p) const;
		void Expand(const Eigen::Vector3f& p);
		void Expand(const AABB& other);
		bool IntersectRay(const Ray& ray, float& tNear, float& tFar) const;
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

		std::vector<std::pair<int, int>> sortedClusters;

		void Clear();
		void Resize(size_t newSize);
		[[nodiscard]] PointCloud Clone() const;
		void CopyFrom(const PointCloud& src);
		void CopyTo(PointCloud& dst) const;
		void FromPLY(const std::string& plyFileName);
		void FromPLY(const PLYFormat& ply);
		void ToPLY(const std::string& plyFileName) const;
		void ToPLY(PLYFormat& ply) const;
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

		uint64_t GetKey(int x, int y, int z) const;

		Eigen::Vector3i GetIndex(const Eigen::Vector3f& position) const;

		void Build(const GeometricProcessingPipeline::PointCloud& pc, float cellSize);

		int GetClosestPoint(const std::vector<Eigen::Vector3f>& points, const Eigen::Vector3f& queryPos, float& outDist);

		void GetKNearestNeighbors(
			const std::vector<Eigen::Vector3f>& points,
			const Eigen::Vector3f& queryPos,
			int k,
			std::vector<unsigned int>& outIndices,
			std::vector<float>& outDistances);

		SparseGridPickResult Pick(const std::vector<Eigen::Vector3f>& points, const Eigen::Ray& ray, float pickRadius);

		SparseGridPickResult PickBruteForce(const std::vector<Eigen::Vector3f>& points, const Eigen::Ray& ray, float pickRadius);

		void Visualize(const GeometricProcessingPipeline::PointCloud& pc);
	};

	typedef uint64_t DataBlockKey;

	struct DataBlock
	{
		Eigen::Vector3f blockMin = Eigen::Vector3f::Zero();
		Voxel voxels[Configuration::voxelsPerBlock];
		std::mutex blockMutex;

		void Initialize();
	};

	struct SparseDataBlock
	{
		float voxelSize = Configuration::voxelSize;
		Eigen::Vector3f gridOrigin = Eigen::Vector3f::Zero();
		std::unordered_map<DataBlockKey, std::unique_ptr<DataBlock>> dataBlocks;

		float blockSizePerAxis = voxelSize * Configuration::voxelsPerBlockAxis;

		Voxel* GetVoxelByIndex(int gx, int gy, int gz);

		void FromPointsData(const std::vector<Eigen::Vector3f>& points,
			const std::vector<Eigen::Vector3f>& normals,
			const std::vector<Eigen::Vector3f>& colors,
			const std::vector<int>& clusterIds,
			const Eigen::Vector3f& aabbMin);

		void Visualize();
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

		void Generate(SparseDataBlock& sdb);

		void Visualize(bool showMesh, bool showHoles);

		void ExportPLY(const std::string& filename);

		void DetectHoles();
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

		virtual void Process(int pipelineIndex) = 0;
		void Process(int pipelineIndex, SpatialPartitioningType* spatialPartitioning)
		{
			this->spatialPartitioning = spatialPartitioning;
			this->pipelineIndex = pipelineIndex;
			Process(pipelineIndex);
		}

		virtual void Visualize() = 0;

		inline Pipeline* GetPipeline() const { return pipeline; }

		inline SpatialPartitioningType* GetSpatialPartitioning() const { return spatialPartitioning; }

		inline GeometricProcessingOperatorParameter& GetParameter() { return parameter; }

		inline int GetPipelineIndex() const { return pipelineIndex; }

		inline void SetGeometricProcessingOperatorParameter(const GeometricProcessingOperatorParameter& param) { parameter = param; }
		inline const std::vector<int>& GetPointTags() const { return pointTags; }
		inline void SetPointTags(const std::vector<int>& tags) { pointTags = tags; }

	protected:
		Pipeline* pipeline = nullptr;
		GeometricProcessingOperatorParameter parameter;
		int pipelineIndex = -1;
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
		std::shared_ptr<OperatorType> AddOperator(const std::string& tag)
		{
			auto op = std::make_shared<OperatorType>(this, GeometricProcessingOperatorParameter());
			operators.emplace_back(std::make_tuple(tag, op));
			return op;
		}

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
			if (-1 == currentPointCloudIndex || currentPointCloudIndex >= pointClouds.size()) return nullptr;
			else return pointClouds[currentPointCloudIndex];
		}

		inline std::shared_ptr<PointCloud> GetInitialPointCloud() { return pointClouds.empty() ? nullptr : pointClouds[0]; }
		inline std::shared_ptr<PointCloud> GetLastPointCloud() { return pointClouds.empty() ? nullptr : pointClouds.back(); }
		inline std::shared_ptr<PointCloud> GetPointCloud(int index)
		{
			if (-1 == index) return GetLastPointCloud();
			else if (pointClouds.empty()) return nullptr;
			else return pointClouds[index];
		}

		inline std::shared_ptr<IGeometricProcessingOperator<SparseGrid>> GetOperator(int index)
		{
			if (-1 == index)
			{
				auto& [opTag, op] = operators.back();
				return op;
			}
			else if (index >= operators.size())
			{
				return nullptr;
			}
			else
			{
				auto& [opTag, op] = operators[index];
				return op;
			}
		}

	protected:
		std::vector<std::tuple<std::string, std::shared_ptr<IGeometricProcessingOperator<SparseGrid>>>> operators;
		SparseGrid* sparseGrid = nullptr;
		std::vector<Triangle> generatedMeshTriangles;

		std::vector<std::shared_ptr<PointCloud>> pointClouds;
		int currentPointCloudIndex = -1;
	};

	template<typename Functor>
	class OperatorCustom : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCustom(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process(int pipelineIndex) override
		{
			cachedPointCloud = pipeline->GetCurrentPointCloud();

			Functor func;
			func(this, *cachedPointCloud);
		}

		void Visualize()
		{
			for (size_t i = 0; i < cachedPointCloud->numberOfElements; i++)
			{
				const auto& p = cachedPointCloud->positions[i];
				const auto& c = cachedPointCloud->colors[i];

				VD::AddSphere("OperatorCustom",
					p,
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f)
				);
			}
		}
	};

	class OperatorPointCloudLoader : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudLoader(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline const std::string& GetPLYFilename() const { return plyFilename; }

		
		inline OperatorPointCloudLoader* SetPLYFilename(const std::string& filename) { plyFilename = filename; return this; }

	protected:
		std::string plyFilename;
	};

	class OperatorPointCloudSaver : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudSaver(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline const std::string& GetPLYFilename() const { return plyFilename; }

		
		inline OperatorPointCloudSaver* SetPLYFilename(const std::string& filename) { plyFilename = filename; return this; }

	protected:
		std::string plyFilename;
	};

	class OperatorStorePointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorStorePointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorRestoreInitialPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorRestoreInitialPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorRestoreLastPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorRestoreLastPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorShowMarks : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorShowMarks(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);
		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorExpandMarks : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorExpandMarks(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorExpandMarks* SetTargetMarkName(const std::string& name) { targetMarkName = name; return this; }
		OperatorExpandMarks* SetIterations(int iter) { iterations = iter; return this; }
		OperatorExpandMarks* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }

	private:
		std::string targetMarkName = "OperatorNormalDivergence";
		int iterations = 1;
		int neighborSearchOffset = 1;
	};

	class OperatorPointCloudVisualization : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudVisualization(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorSOR : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorSOR(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Method Chaining Setters]
		OperatorSOR* SetKNeighbors(int k) { kNeighbors = k; return this; }
		OperatorSOR* SetStdDevMultiplier(float mult) { stdDevMultiplier = mult; return this; }
		OperatorSOR* SetRemoveOutliers(bool remove) { removeOutliers = remove; return this; }

	private:
		// Parameters
		int kNeighbors = 50;           // 주변 이웃 개수 (분석용)
		float stdDevMultiplier = 1.0f; // 임계값 계수 (Mean + n * StdDev)
		bool removeOutliers = false;    // true면 실제 데이터 삭제, false면 마킹만

		// Statistics & Data
		std::vector<float> pointMeanDistances;
		std::vector<int> outlierIndices; // 시각화용
		float globalMean = 0.0f;
		float globalStdDev = 0.0f;
		float distanceThreshold = 0.0f;
	};

	class OperatorPointCloudDensity : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudDensity(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorPointCloudDensity* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorPointCloudDensity* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorPointCloudDensity* SetRangeMin(float minVal) { range_min = minVal; return this; }
		OperatorPointCloudDensity* SetRangeMax(float maxVal) { range_max = maxVal; return this; }

	private:
		std::vector<float> pointDensities;

		float densityMean = 0.0f;
		float densityStdDev = 0.0f;

		float searchRadiusMultiplier = 1.5f;
		int neighborSearchOffset = 1;

		float range_min = 0.0f;
		float range_max = 1.0f;
	};

	class OperatorMeanShift : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorMeanShift(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorMeanShift* SetBandwidthMultiplier(float mult) { bandwidthMultiplier = mult; return this; }
		OperatorMeanShift* SetConvergenceThreshold(float th) { convergenceThreshold = th; return this; }
		OperatorMeanShift* SetMaxIterations(int iter) { maxIterations = iter; return this; }
		OperatorMeanShift* SetUpdatePositions(bool update) { updatePositions = update; return this; }
		OperatorMeanShift* SetVisualizationScale(float scale) { visualizationScale = scale; return this; }
		OperatorMeanShift* SetInvertDirection(bool invert) { invertDirection = invert; return this; }
		OperatorMeanShift* SetMarkedPointsOnly(bool markedOnly) { markedPointsOnly = markedOnly; return this; }

	private:
		std::vector<Eigen::Vector3f> shiftedPositions;
		std::vector<float> shiftDistances;

		float bandwidthMultiplier = 5.0f;
		float convergenceThreshold = 0.01f;
		int maxIterations = 10;
		bool updatePositions = true;
		float visualizationScale = 1.0f;
		bool invertDirection = false;
		bool markedPointsOnly = false;
	};

	class OperatorPointCloudLaplacianSmoothing : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudLaplacianSmoothing(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline int GetIterations() const { return iterations; }
		inline float GetSmoothingFactor() const { return smoothingFactor; }
		inline float GetSearchRadiusMultiplier() const { return searchRadiusMultiplier; }
		inline int GetNeighborSearchOffset() const { return neighborSearchOffset; }
		inline bool IsPreserveMarks() const { return preserveMarks; }

		// [Setters]
		OperatorPointCloudLaplacianSmoothing* SetIterations(int iter) { iterations = iter; return this; }
		OperatorPointCloudLaplacianSmoothing* SetSmoothingFactor(float lambda) { smoothingFactor = std::clamp(lambda, 0.0f, 1.0f); return this; }
		OperatorPointCloudLaplacianSmoothing* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorPointCloudLaplacianSmoothing* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorPointCloudLaplacianSmoothing* SetPreserveMarks(bool preserve) { preserveMarks = preserve; return this; }

	private:
		int iterations = 3;
		float smoothingFactor = 0.5f;
		float searchRadiusMultiplier = 1.5f;
		int neighborSearchOffset = 1;
		bool preserveMarks = true;
	};

	class OperatorKNNSmoothing : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorKNNSmoothing(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline int GetK() const { return kNeighbors; }
		inline int GetIterations() const { return iterations; }
		inline float GetSmoothingFactor() const { return smoothingFactor; }

		// [Setters]
		OperatorKNNSmoothing* SetK(int k) { kNeighbors = k; return this; }
		OperatorKNNSmoothing* SetIterations(int iter) { iterations = iter; return this; }
		OperatorKNNSmoothing* SetSmoothingFactor(float factor) { smoothingFactor = std::clamp(factor, 0.0f, 1.0f); return this; }
		OperatorKNNSmoothing* SetPreserveMarks(bool preserve) { preserveMarks = preserve; return this; }

	private:
		int kNeighbors = 8;
		int iterations = 3;
		float smoothingFactor = 0.5f;
		bool preserveMarks = true;
	};

	class OperatorSurfaceFitting : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorSurfaceFitting(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorSurfaceFitting* SetKNeighbors(int k) { kNeighbors = k; return this; }
		OperatorSurfaceFitting* SetIteration(int iter) { iteration = iter; return this; }
		OperatorSurfaceFitting* SetUpdateNormals(bool update) { updateNormals = update; return this; }
		OperatorSurfaceFitting* SetPreserveMarks(bool preserve) { preserveMarks = preserve; return this; }

	private:
		int kNeighbors = 32;
		int iteration = 10;
		bool updateNormals = true;
		bool preserveMarks = true;

		void ComputeEigenDecomposition(const Eigen::Matrix3f& cov, Eigen::Vector3f& outEvals, Eigen::Matrix3f& outEvecs);
	};

	class OperatorPointDensity : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointDensity(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline float GetSearchRadiusScale() const { return searchRadiusScale; }
		inline int GetNeighborSearchOffset() const { return neighborSearchOffset; }
		inline float GetMinDensity() const { return minDensity; }
		inline float GetMaxDensity() const { return maxDensity; }

		// [Setters]
		OperatorPointDensity* SetSearchRadiusScale(float scale) { searchRadiusScale = scale; return this; }
		OperatorPointDensity* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }

	private:
		std::vector<float> densities;
		int neighborSearchOffset = 1;
		float searchRadiusScale = 2.0f;
		float minDensity = 0.0f;
		float maxDensity = 0.0f;
	};

	class OperatorFindOverlappingPoints : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorFindOverlappingPoints(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline float GetMaxNormalAngle() const { return maxNormalAngle; }
		inline int GetOverlappingCount() const { return overlappingCount; }

		// [Setters]
		OperatorFindOverlappingPoints* SetMaxNormalAngle(float degrees) { maxNormalAngle = degrees; return this; }
		OperatorFindOverlappingPoints* SetOverlapDistanceThreshold(float threshold) { overlapDistanceThreshold = threshold; return this; }

	private:
		float overlapDistanceThreshold = 1e-4f;
		float maxNormalAngle = 15.0f;

		int overlappingCount = 0;
	};

#pragma region Morphological
	class MorphologyHelper
	{
	public:
		// mode: 0 = Erosion (Min), 1 = Dilation (Max)
		static void ApplyMorphology(
			std::shared_ptr<PointCloud> cloud,
			SparseGrid* grid,
			float radius,
			int mode,
			int neighborSearchOffset = 1);

		static void MarkOverlappingPointsAbove(
			std::shared_ptr<PointCloud> cloud,
			SparseGrid* grid,
			float overlapDistThreshold = 1e-4f,
			float maxAngle = 15.0f);
	};

	class OperatorApplyMorphology : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorApplyMorphology(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorApplyMorphology* SetOperation(const std::string& op) { operation = op; return this; }
		OperatorApplyMorphology* SetRadius(float r) { radius = r; return this; }
		OperatorApplyMorphology* SetOverlapDist(float dist) { overlapDist = dist; return this; }
		OperatorApplyMorphology* SetMaxAngle(float angle) { maxAngle = angle; return this; }
		OperatorApplyMorphology* SetMarkingOnly(bool marking) { markingOnly = marking; return this; }

	private:
		std::string operation = "Erosion";
		float radius = 0.05f;
		float overlapDist = 1e-4f;
		float maxAngle = 15.0f;
		bool markingOnly = false;
	};

	class OperatorErosionAndClustering : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorErosionAndClustering(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorErosionAndClustering* SetErosionRadius(float r) { erosionRadius = r; return this; }
		OperatorErosionAndClustering* SetErosionIterations(int iter) { erosionIterations = iter; return this; }
		OperatorErosionAndClustering* SetClusterDistance(float dist) { clusterDistance = dist; return this; }
		OperatorErosionAndClustering* SetMinClusterSize(int size) { minClusterSize = size; return this; }
		OperatorErosionAndClustering* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorErosionAndClustering* SetVisualizeErodedOnly(bool vis) { visualizeErodedOnly = vis; return this; }

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

	private:
		float erosionRadius = 0.05f;
		int erosionIterations = 5;
		float clusterDistance = 0.06f;
		int minClusterSize = 50;
		int neighborSearchOffset = 3;
		bool visualizeErodedOnly = false;

		std::vector<Eigen::Vector3f> erodedPositions;
	};
#pragma endregion

#pragma region About Normal
	class OperatorNormalDeviation : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalDeviation(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		void VisualizeDefault();
		void VisualizeHeatmap();

		inline const std::vector<float>& GetDeviations() const { return deviations; }

		// [Setters]
		OperatorNormalDeviation* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorNormalDeviation* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorNormalDeviation* SetMaxDeviationAngle(float angle) { maxDeviationAngle = angle; return this; }

	private:
		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;
		float maxDeviationAngle = 30.0f;

		std::vector<float> deviations;
	};

	class OperatorNormalGradient : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalGradient(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline float GetGradientMean() const { return gradientMean; }
		inline float GetGradientStdDev() const { return gradientStdDev; }

		// [Setters]
		OperatorNormalGradient* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorNormalGradient* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorNormalGradient* SetVisualizationSigma(float sigma) { visualizationSigma = sigma; return this; }
		OperatorNormalGradient* SetUseAlphaGradient(bool use) { useAlphaGradient = use; return this; }

	private:
		std::vector<float> gradients;

		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;
		bool useAlphaGradient = false;

		float visualizationSigma = 3.0f;
		float gradientMean = 0.0f;
		float gradientStdDev = 0.0f;
	};

	class OperatorNormalVariance : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalVariance(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline float GetVarianceMean() const { return varianceMean; }
		inline float GetVarianceStdDev() const { return varianceStdDev; }

		// [Setters]
		OperatorNormalVariance* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorNormalVariance* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorNormalVariance* SetVisualizationSigma(float sigma) { visualizationSigma = sigma; return this; }

	private:
		// 결과 데이터: 각 점의 법선 분산값
		std::vector<float> normalVariances;

		// 파라미터
		float searchRadiusMultiplier = 1.5f;
		int neighborSearchOffset = 1;
		float visualizationSigma = 3.0f;

		// 통계
		float varianceMean = 0.0f;
		float varianceStdDev = 0.0f;
	};

	class OperatorNormalDivergence : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalDivergence(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorNormalDivergence* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorNormalDivergence* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }

	private:
		std::vector<float> normalDivergences;
		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;
		float minDivergence = 0.0f;
		float maxDivergence = 0.0f;
	};

	class OperatorNormalDivergenceGradient : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalDivergenceGradient(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline float GetGradientMean() const { return gradientMean; }
		inline float GetGradientStdDev() const { return gradientStdDev; }

		// [Setters]
		OperatorNormalDivergenceGradient* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorNormalDivergenceGradient* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorNormalDivergenceGradient* SetVisualizationSigma(float sigma) { visualizationSigma = sigma; return this; }

	private:
		std::vector<float> normalDivergences;
		std::vector<float> divergenceGradients;

		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;

		float visualizationSigma = 3.0f;
		float gradientMean = 0.0f;
		float gradientStdDev = 0.0f;
	};
#pragma endregion

	template<typename IterateFunctor>
	class OperatorPointCloudIterator : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudIterator(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process(int pipelineIndex) override
		{
			TS(PointCloudIterator);

			auto currentPointCloud = pipeline->GetCurrentPointCloud();
			if (currentPointCloud->numberOfElements == 0) return;

			cachedPointCloud = currentPointCloud;

			IterateFunctor iterateFunctor;

			for (size_t i = 0; i < currentPointCloud->numberOfElements; i++)
			{
				iterateFunctor(this, *currentPointCloud, i);
			}

			TE(PointCloudIterator);
		}

		void Visualize()
		{
			auto numberOfPoints = cachedPointCloud->numberOfElements;
			for (size_t i = 0; i < numberOfPoints; i++)
			{
				auto& p = cachedPointCloud->positions[i];
				auto& n = cachedPointCloud->normals[i].normalized();
				auto& c = cachedPointCloud->colors[i];
				VD::AddSphere("PointCloudIterator", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f));
			}
		}
	};

	template<typename FilterFunctor>
	class OperatorCustomFilter : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCustomFilter(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		void Process(int pipelineIndex) override
		{
			TS(CustomFilter);

			auto currentPointCloud = pipeline->GetCurrentPointCloud();
			if (currentPointCloud->numberOfElements == 0) return;

			size_t writeIdx = 0;
			FilterFunctor filterFunctor;

			bool hasClassIDs = !currentPointCloud->pointDeepLearningClassIDs.empty();
			bool hasClusterIDs = !currentPointCloud->pointClusterIDs.empty();
			bool hasMarks = !currentPointCloud->marks.empty();

			for (size_t readIdx = 0; readIdx < currentPointCloud->numberOfElements; ++readIdx)
			{
				if (filterFunctor(this, *currentPointCloud, readIdx))
				{
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
						{
							for (auto& [k, v] : currentPointCloud->marks)
							{
								v[writeIdx] = v[readIdx];
							}
						}
					}
					writeIdx++;
				}
			}

			printf("CustomFilter: %zu points removed\n", currentPointCloud->numberOfElements - writeIdx);

			currentPointCloud->numberOfElements = writeIdx;
			currentPointCloud->positions.resize(writeIdx);
			currentPointCloud->normals.resize(writeIdx);
			currentPointCloud->colors.resize(writeIdx);

			if (hasClassIDs) currentPointCloud->pointDeepLearningClassIDs.resize(writeIdx);
			if (hasClusterIDs) currentPointCloud->pointClusterIDs.resize(writeIdx);
			if (hasMarks)
			{
				for (auto& [k, v] : currentPointCloud->marks)
				{
					v.resize(writeIdx);
				}
			}

			cachedPointCloud = currentPointCloud;

			this->parameter.needToRebuildSpatialPartitioning = true;

			TE(CustomFilter);
		}

		void Visualize()
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
	};

	class OperatorFilterLeaveLargestOnly : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorFilterLeaveLargestOnly(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorFilterMarked : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorFilterMarked(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorFilterUnmarked : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorFilterUnmarked(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorFilterETC : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorFilterETC(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorLocalPlaneFitting : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorLocalPlaneFitting(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline Eigen::Vector3f GetFittedPoint(int index) const
		{
			if (index >= 0 && index < fittedPoints.size()) return fittedPoints[index];
			return Eigen::Vector3f::Zero();
		}

		inline std::vector<Eigen::Vector3f>& GetFiitedPoints() { return fittedPoints; }
		inline const std::vector<Eigen::Vector3f>& GetFiitedPoints() const { return fittedPoints; }

		// [Setters]
		OperatorLocalPlaneFitting* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorLocalPlaneFitting* SetUpdatePositions(bool update) { updatePositions = update; return this; }
		OperatorLocalPlaneFitting* SetUpdateThreshold(float threshold) { updateThreshold = threshold; return this; }
		OperatorLocalPlaneFitting* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorLocalPlaneFitting* SetMarkedPointsOnly(bool markedOnly) { markedPointsOnly = markedOnly; return this; }

	private:
		float searchRadiusMultiplier = 2.0f;
		bool updatePositions = false;
		float updateThreshold = 0.0f;
		bool markedPointsOnly = false;

		int neighborSearchOffset = 3;

		float updateRatio = 0.5f;

		std::vector<float> fittingResiduals;
		std::vector<Eigen::Vector3f> fittedPoints;
		float residualMean = 0.0f;
		float residualStdDev = 0.0f;
	};

	class OperatorCompareSameIndexOrderedPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCompareSameIndexOrderedPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorCompareSameIndexOrderedPointCloud* SetComparisonDistanceThreshold(float dist) { comparisonDistanceThresholdSq = dist * dist; return this; }
		OperatorCompareSameIndexOrderedPointCloud* SetIndexA(int idx) { indexA = idx; return this; }
		OperatorCompareSameIndexOrderedPointCloud* SetIndexB(int idx) { indexB = idx; return this; }

	private:
		std::vector<char> matchedFlags;
		std::shared_ptr<PointCloud> pointCloudA;
		std::shared_ptr<PointCloud> pointCloudB;
		float comparisonDistanceThresholdSq = 1e-5f;
		int deletedCount = 0;
		int indexA = 0;
		int indexB = -1;
	};

	class OperatorComparePointCloudUsingDistance : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorComparePointCloudUsingDistance(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorComparePointCloudUsingDistance* SetComparisonDistanceThreshold(float t) { comparisonDistanceThreshold = t; return this; }
		OperatorComparePointCloudUsingDistance* SetIndexA(int idx) { indexA = idx; return this; }
		OperatorComparePointCloudUsingDistance* SetIndexB(int idx) { indexB = idx; return this; }

	private:
		float comparisonDistanceThreshold = 0.0000001f;

		std::vector<char> matchedFlags;

		std::shared_ptr<PointCloud> pointCloudA;
		std::shared_ptr<PointCloud> pointCloudB;
		int indexA = 0;
		int indexB = -1;
	};

	class OperatorCompareWithLastPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCompareWithLastPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline float GetComparisonDistanceThreshold() const { return comparisonDistanceThreshold; }

		
		inline OperatorCompareWithLastPointCloud* SetComparisonDistanceThreshold(float threshold) { comparisonDistanceThreshold = threshold; return this; }

	private:
		float comparisonDistanceThreshold = 0.0001f;
		std::vector<bool> matchedFlags;
	};

	class OperatorClustering : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorClustering(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

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

			int Find(int i)
			{
				int p = parent[i].load(std::memory_order_relaxed);
				while (p != i)
				{
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

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		inline bool IsUseMarksForClustering() const { return useMarksForClustering; }

		inline OperatorClustering* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		inline OperatorClustering* SetUseMarksForClustering(bool useMarks) { useMarksForClustering = useMarks; return this; }

	protected:
		float searchRadiusMultiplier = 0.5f;
		bool useMarksForClustering = true;
	};

	class OperatorClusterBorderFinding : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorClusterBorderFinding(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

	protected:
		std::vector<int> borderPointIndices;
		std::mutex borderIndicesMutex;
	};

	class OperatorClusteringComplex : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorClusteringComplex(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		struct ClusteringParams
		{
			float searchRadiusMult = 1.5f;
			float angleThreshold = 0.9f;
			float planeOffsetThreshold = 0.2f;
			float colorThreshold = 0.15f;
			float curvatureDiffThreshold = 0.05f;
			bool useDeepLearningClasses = true;
		};

		ClusteringParams params;

		// [Setters]
		OperatorClusteringComplex* SetSearchRadiusMult(float val) { params.searchRadiusMult = val; return this; }
		OperatorClusteringComplex* SetAngleThreshold(float val) { params.angleThreshold = val; return this; }
		OperatorClusteringComplex* SetPlaneOffsetThreshold(float val) { params.planeOffsetThreshold = val; return this; }
		OperatorClusteringComplex* SetColorThreshold(float val) { params.colorThreshold = val; return this; }
		OperatorClusteringComplex* SetCurvatureDiffThreshold(float val) { params.curvatureDiffThreshold = val; return this; }
		OperatorClusteringComplex* SetUseDeepLearningClasses(bool val) { params.useDeepLearningClasses = val; return this; }

		std::vector<int> pointClusterIds;
		std::vector<float> pointCurvatures;

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

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
	};

	class OperatorCurvatureEstimation : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCurvatureEstimation(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorCurvatureEstimation* SetCurvatureThreshold(float th) { curvatureThreshold = th; return this; }
		OperatorCurvatureEstimation* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorCurvatureEstimation* SetSearchRadiusScale(float scale) { searchRadiusScale = scale; return this; }

	private:
		float curvatureThreshold = 0.1f;
		int neighborSearchOffset = 3;
		float searchRadiusScale = 5.0f;

		std::vector<float> curvatures;

		inline Eigen::Vector3f ComputeEigenValuesSymmetric(const Eigen::Matrix3f& M)
		{
			double m = (M(0, 0) + M(1, 1) + M(2, 2)) / 3.0;
			double p = (std::pow(M(0, 0) - m, 2.0) + std::pow(M(1, 1) - m, 2.0) + std::pow(M(2, 2) - m, 2.0) +
				2.0 * (std::pow(M(0, 1), 2.0) + std::pow(M(0, 2), 2.0) + std::pow(M(1, 2), 2.0))) / 6.0;

			double q = (M - Eigen::Matrix3f::Identity() * m).determinant() / 2.0;
			double phi = 0.0;

			if (p > 1e-9)
			{
				phi = std::atan2(std::sqrt(std::max(0.0, p * p * p - q * q)), q) / 3.0;
			}

			if (phi < 0) phi += 3.14159265358979323846 / 3.0;

			double eig1 = m + 2.0 * std::sqrt(p) * std::cos(phi);
			double eig2 = m + 2.0 * std::sqrt(p) * std::cos(phi + 2.0 * 3.14159265358979323846 / 3.0);
			double eig3 = 3.0 * m - eig1 - eig2;

			return Eigen::Vector3f((float)eig1, (float)eig2, (float)eig3);
		}
	};

	class OperatorCurvatureEstimationAppliedNormal : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCurvatureEstimationAppliedNormal(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorCurvatureEstimationAppliedNormal* SetCurvatureThreshold(float th) { curvatureThreshold = th; return this; }
		OperatorCurvatureEstimationAppliedNormal* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorCurvatureEstimationAppliedNormal* SetSearchRadiusScale(float scale) { searchRadiusScale = scale; return this; }
		OperatorCurvatureEstimationAppliedNormal* SetVisualScale(float scale) { visualScale = scale; return this; }

	private:
		float curvatureThreshold = 0.1f;
		int neighborSearchOffset = 3;
		float searchRadiusScale = 5.0f;
		float visualScale = 5.0f;

		std::vector<float> curvatures;

		inline Eigen::Vector3f ComputeEigenValuesSymmetric(const Eigen::Matrix3f& M)
		{
			double m = (M(0, 0) + M(1, 1) + M(2, 2)) / 3.0;
			double p = (std::pow(M(0, 0) - m, 2.0) + std::pow(M(1, 1) - m, 2.0) + std::pow(M(2, 2) - m, 2.0) +
				2.0 * (std::pow(M(0, 1), 2.0) + std::pow(M(0, 2), 2.0) + std::pow(M(1, 2), 2.0))) / 6.0;

			double q = (M - Eigen::Matrix3f::Identity() * m).determinant() / 2.0;
			double phi = 0.0;

			if (p > 1e-9)
			{
				phi = std::atan2(std::sqrt(std::max(0.0, p * p * p - q * q)), q) / 3.0;
			}

			if (phi < 0) phi += 3.14159265358979323846 / 3.0;

			double eig1 = m + 2.0 * std::sqrt(p) * std::cos(phi);
			double eig2 = m + 2.0 * std::sqrt(p) * std::cos(phi + 2.0 * 3.14159265358979323846 / 3.0);
			double eig3 = 3.0 * m - eig1 - eig2;

			return Eigen::Vector3f((float)eig1, (float)eig2, (float)eig3);
		}
	};

	class OperatorCurvatureDivergence : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCurvatureDivergence(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		// [Setters]
		OperatorCurvatureDivergence* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorCurvatureDivergence* SetVisualizationScale(float scale) { visualizationScale = scale; return this; }

	private:
		std::vector<float> curvatures;
		std::vector<Eigen::Vector3f> gradients;
		std::vector<float> divergences;

		float searchRadiusMultiplier = 2.0f;
		float visualizationScale = 500.0f;

		template<typename Func>
		inline void ProcessNeighbors(int idx, const Eigen::Vector3f& p, const SparseGrid& sg, float rSq, Func func)
		{
			int gx = (int)std::floor((p.x() - sg.aabb.min.x()) / sg.cellSize);
			int gy = (int)std::floor((p.y() - sg.aabb.min.y()) / sg.cellSize);
			int gz = (int)std::floor((p.z() - sg.aabb.min.z()) / sg.cellSize);

			for (int dz = -1; dz <= 1; ++dz) {
				for (int dy = -1; dy <= 1; ++dy) {
					for (int dx = -1; dx <= 1; ++dx) {
						uint64_t key = sg.GetKey(gx + dx, gy + dy, gz + dz);
						auto it = sg.voxelPointListHead.find(key);
						if (it == sg.voxelPointListHead.end()) continue;

						int curr = it->second;
						while (curr != -1) {
							if (curr != idx) {
								if ((p - cachedPointCloud->positions[curr]).squaredNorm() <= rSq) {
									func(curr);
								}
							}
							curr = sg.nextPoint[curr];
						}
					}
				}
			}
		}

		inline Eigen::Vector3f ComputeEigenValuesSymmetric(const Eigen::Matrix3f& M)
		{
			double m = (M(0, 0) + M(1, 1) + M(2, 2)) / 3.0;
			double p = (std::pow(M(0, 0) - m, 2.0) + std::pow(M(1, 1) - m, 2.0) + std::pow(M(2, 2) - m, 2.0) +
				2.0 * (std::pow(M(0, 1), 2.0) + std::pow(M(0, 2), 2.0) + std::pow(M(1, 2), 2.0))) / 6.0;

			double q = (M - Eigen::Matrix3f::Identity() * m).determinant() / 2.0;
			double phi = 0.0;
			if (p > 1e-9) phi = std::atan2(std::sqrt(std::max(0.0, p * p * p - q * q)), q) / 3.0;
			if (phi < 0) phi += 3.14159265358979323846 / 3.0;

			double eig1 = m + 2.0 * std::sqrt(p) * std::cos(phi);
			double eig2 = m + 2.0 * std::sqrt(p) * std::cos(phi + 2.0 * 3.14159265358979323846 / 3.0);
			double eig3 = 3.0 * m - eig1 - eig2;
			return Eigen::Vector3f((float)eig1, (float)eig2, (float)eig3);
		}
	};

	class OperatorCurvatureDeviation : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCurvatureDeviation(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;
		void VisualizeHeatmap();
		void VisualizeMarker();

		inline float GetDeviationMean() const { return deviationMean; }
		inline float GetDeviationStdDev() const { return deviationStdDev; }
		inline const std::vector<float>& GetDeviations() const { return deviations; }

		// [Method Chaining Setters]
		OperatorCurvatureDeviation* SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; return this; }
		OperatorCurvatureDeviation* SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; return this; }
		OperatorCurvatureDeviation* SetVisualizationSigma(float sigma) { visualizationSigma = sigma; return this; }
		OperatorCurvatureDeviation* SetDeviationThreshold(float threshold) { deviationThreshold = threshold; return this; }

	private:
		// Parameters
		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;
		float visualizationSigma = 3.0f;

		// Data
		std::vector<float> curvatures;
		std::vector<float> deviations;

		// Statistics
		float deviationMean = 0.0f;
		float deviationStdDev = 0.0f;
		float deviationThreshold = 0.5f;

		// Helper
		inline Eigen::Vector3f ComputeEigenValuesSymmetric(const Eigen::Matrix3f& M);
	};

	class OperatorMeshGeneration : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorMeshGeneration(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		void ExportPLY(const std::string& filename);

		inline float GetMeshVoxelSize() const { return meshVoxelSize; }

		inline std::vector<Triangle>& GetTriangles() { return meshGenerator.triangles; }
		inline const std::vector<Triangle>& GetTriangles() const { return meshGenerator.triangles; }

		// [Setters]
		OperatorMeshGeneration* SetMeshVoxelSize(float size) { meshVoxelSize = size; return this; }
		OperatorMeshGeneration* SetExportFilename(const std::string& filename) { exportFilename = filename; return this; }
		OperatorMeshGeneration* SetShowMesh(bool show) { showMesh = show; return this; }
		OperatorMeshGeneration* SetShowHoles(bool show) { showHoles = show; return this; }
		OperatorMeshGeneration* SetDetectHoles(bool detect) { detectHoles = detect; return this; }

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
		OperatorMeshDistanceFilter(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter);

		virtual void Process(int pipelineIndex) override;
		virtual void Visualize() override;

		void SetReferenceMesh(const std::vector<Triangle>& meshTriangles);

		inline float GetAverageDistance() const { return averageDistance; }

		
		inline OperatorMeshDistanceFilter* SetThresholdMultiplier(float mult) { thresholdMultiplier = mult; return this; }

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
		Eigen::Vector3f triGridMin = Eigen::Vector3f::Zero();

		void BuildTriangleGrid();

		float GetClosestDistanceFromMesh(const Eigen::Vector3f& p);

		float SqDistPointTriangle(const Eigen::Vector3f& p, const Triangle& tri);
	};
}