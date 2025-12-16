#pragma once

#include "GPP_Common.hpp"

namespace GeometricProcessingPipeline
{
    class OperatorPointCloudLoader : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudLoader(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process(PointCloud* currentPointCloud) override
		{
			plyFilename = parameter.GetParameter<std::string>("plyFilename", "");

            if (plyFilename.empty()) return;

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
				spatialPartitioning = new SparseGrid();
				spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
			}
			cachedPointCloud = currentPointCloud;
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

    class OperatorStorePointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorStorePointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process(PointCloud* currentPointCloud) override
		{
            TS(StorePointCloud);
			pipeline->StorePointCloud();
			TE(StorePointCloud);
		}
		
        virtual void Visualize() override
        {
			// Do nothing
		}
	};

    class OperatorRestoreInitialPointCloud : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorRestoreInitialPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(StorePointCloud);
            pipeline->RestoreInitialPointCloud();
            TE(StorePointCloud);
        }

        virtual void Visualize() override
        {
            // Do nothing
        }
    };

    class OperatorRestoreLastPointCloud : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorRestoreLastPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(StorePointCloud);
            pipeline->RestoreLastPointCloud();
            TE(StorePointCloud);
        }

        virtual void Visualize() override
        {
            // Do nothing
        }
    };

    class OperatorPointCloudVisualization : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorPointCloudVisualization(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }
        
        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(PointCloudVisualization);
            if (currentPointCloud->numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }
            cachedPointCloud = currentPointCloud;
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
                    glm::vec4(cachedPointCloud->colors[i], 1.0f)

                    //Color::red()
                );
            }
        }
	};

    class OperatorPointCloudLaplacianSmoothing : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorPointCloudLaplacianSmoothing(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(LaplacianSmoothing);

            if (currentPointCloud->numberOfElements == 0) return;

            // 1. Build Spatial Partitioning (한 번 빌드하여 모든 반복에 사용)
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = currentPointCloud;
            size_t numPoints = currentPointCloud->numberOfElements;

            float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // 위치 핑퐁(Ping-Pong)을 위한 버퍼
            std::vector<glm::vec3> nextPositions = currentPointCloud->positions;

            // 2. Iterative Smoothing
            for (int iter = 0; iter < iterations; ++iter)
            {
                // 병렬 처리: 각 점에 대해 이웃의 평균 위치 계산
                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                    {
                        // 마킹된 포인트(예: 경계점)는 이동하지 않음 (옵션)
                        if (preserveMarks && !currentPointCloud->marks.empty() && currentPointCloud->marks[i] != 0)
                        {
                            nextPositions[i] = currentPointCloud->positions[i];
                            return;
                        }

                        const glm::vec3& p = currentPointCloud->positions[i];
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
                                            // 현재 iteration의 위치 기준이 아닌, 이전 iteration(currentPointCloud->positions) 기준으로 이웃 판단
                                            if (glm::distance2(p, currentPointCloud->positions[curr]) <= searchRadiusSq)
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
                currentPointCloud->positions = nextPositions;
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
        OperatorKNNSmoothing(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(KNNSmoothing);

            if (currentPointCloud->numberOfElements == 0) return;

            // 1. Build Spatial Partitioning if needed (used for KNN search)
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = currentPointCloud;
            size_t numPoints = currentPointCloud->numberOfElements;

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // 위치 업데이트를 위한 Ping-Pong 버퍼
            std::vector<glm::vec3> nextPositions = currentPointCloud->positions;

            // 2. Iterative Smoothing
            for (int iter = 0; iter < iterations; ++iter)
            {
                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                    {
                        // 마킹된 포인트(예: 특징점)는 이동하지 않음 (옵션)
                        //if (preserveMarks && !currentPointCloud->marks.empty() && currentPointCloud->marks[i] != 0)
                        //{
                        //    nextPositions[i] = currentPointCloud->positions[i];
                        //    return;
                        //}

                        if (false == (preserveMarks && !currentPointCloud->marks.empty() && currentPointCloud->marks[i] != 0))
                        {
                            return;
                        }

                        const glm::vec3& p = currentPointCloud->positions[i];

						currentPointCloud->colors[i] = glm::vec3(1.0f, 0.0f, 0.0f);

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
                            currentPointCloud->positions,
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

                                centroid += currentPointCloud->positions[idx];
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
                currentPointCloud->positions = nextPositions;
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
        OperatorSurfaceFitting(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(SurfaceFitting);

            if (currentPointCloud->numberOfElements == 0) return;

            // 1. Build Spatial Partitioning if needed
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = currentPointCloud;
            size_t numPoints = currentPointCloud->numberOfElements;

            for (int i = 0; i < iteration; i++)
            {
                // 결과를 담을 임시 버퍼 (Ping-Pong)
                std::vector<glm::vec3> newPositions = currentPointCloud->positions;
                std::vector<glm::vec3> newNormals = currentPointCloud->normals;

                std::vector<int> indices(numPoints);
                std::iota(indices.begin(), indices.end(), 0);

                // 2. Perform Surface Fitting (Parallel)
                std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                    {
                        // 마킹된 포인트 보호 (옵션)
                        //if (preserveMarks && !currentPointCloud->marks.empty() && currentPointCloud->marks[i] != 0)
                        //{
                        //    return;
                        //}

                        const glm::vec3& p = currentPointCloud->positions[i];

                        // KNN Search
                        // SparseGrid::GetKNearestNeighbors requires these containers
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
                            centroid += currentPointCloud->positions[neighborIndices[j]] * w;
                            totalWeight += w;
                        }

                        if (totalWeight < 1e-6f) return;
                        centroid /= totalWeight;

                        // 2) Compute Weighted Covariance Matrix
                        // Cov = Sum( w_i * (p_i - c) * (p_i - c)^T )
                        float xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

                        for (size_t j = 0; j < k; ++j)
                        {
                            glm::vec3 r = currentPointCloud->positions[neighborIndices[j]] - centroid;
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
                        if (glm::dot(planeNormal, currentPointCloud->normals[i]) < 0.0f)
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
                            currentPointCloud->colors[i] = Color::red();
                        }
                    });

                // 데이터 업데이트
                currentPointCloud->positions = newPositions;
                if (updateNormals)
                {
                    currentPointCloud->normals = newNormals;
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
        OperatorPointDensity(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(PointDensity);

            if (currentPointCloud->numberOfElements == 0) return;

            // 1. Build Spatial Partitioning if needed
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = currentPointCloud;
            size_t numberOfPoints = currentPointCloud->numberOfElements;
            densities.resize(numberOfPoints);

            float searchRadius = spatialPartitioning->cellSize * searchRadiusScale;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numberOfPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // 2. Calculate Density (Parallel)
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = currentPointCloud->positions[i];
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
                                        if (glm::distance2(p, currentPointCloud->positions[currIdx]) <= searchRadiusSq)
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
        OperatorNormalDivergence(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(NormalDivergence);

            if (currentPointCloud->numberOfElements == 0) return;

            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = currentPointCloud;
            size_t numPoints = currentPointCloud->numberOfElements;

            normalDivergences.assign(numPoints, 0.0f);

            float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // Calculate Divergence of Normal Vector Field
            // div(n) approx sum( dot(n_j - n_i, p_j - p_i) / |p_j - p_i|^2 )
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = currentPointCloud->positions[i];
                    const glm::vec3& n_i = currentPointCloud->normals[i];

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
                                        glm::vec3 diff = currentPointCloud->positions[curr] - p;
                                        float distSq = glm::dot(diff, diff);

                                        if (distSq <= searchRadiusSq && distSq > 1e-8f)
                                        {
                                            float dist = std::sqrt(distSq);
                                            glm::vec3 dir = diff / dist;

                                            // Difference in normals
                                            glm::vec3 n_diff = currentPointCloud->normals[curr] - n_i;

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
        OperatorCustomFilter(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(CustomFilter);

            if (currentPointCloud->numberOfElements == 0) return;

            size_t writeIdx = 0;
            FilterFunctor filterFunctor;
            filterFunctor.filter = this;

            bool hasClassIDs = !currentPointCloud->pointDeepLearningClassIDs.empty();
            bool hasClusterIDs = !currentPointCloud->pointClusterIDs.empty();
            bool hasMarks = !currentPointCloud->marks.empty();

            for (size_t readIdx = 0; readIdx < currentPointCloud->numberOfElements; ++readIdx)
            {
                if (filterFunctor(*currentPointCloud, readIdx))
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
                            currentPointCloud->marks[writeIdx] = currentPointCloud->marks[readIdx];
                    }
                    writeIdx++;
                }
            }

            currentPointCloud->numberOfElements = writeIdx;
            currentPointCloud->positions.resize(writeIdx);
            currentPointCloud->normals.resize(writeIdx);
            currentPointCloud->colors.resize(writeIdx);

            if (hasClassIDs) currentPointCloud->pointDeepLearningClassIDs.resize(writeIdx);
            if (hasClusterIDs) currentPointCloud->pointClusterIDs.resize(writeIdx);
            if (hasMarks) currentPointCloud->marks.resize(writeIdx);

            cachedPointCloud = currentPointCloud;

            this->parameter.needToRebuildSpatialPartitioning = true;

            TE(CustomFilter);
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
        OperatorFilterETC(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
		}

        virtual void Process(PointCloud* currentPointCloud) override
        {
            if (currentPointCloud->numberOfElements == 0) return;

            TS(FilterETC);

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
                        currentPointCloud->marks[writeIdx] = currentPointCloud->marks[readIdx];
                }
                writeIdx++;
            }

            currentPointCloud->numberOfElements = writeIdx;
            currentPointCloud->positions.resize(writeIdx);
            currentPointCloud->normals.resize(writeIdx);
            currentPointCloud->colors.resize(writeIdx);

            if (hasClassIDs) currentPointCloud->pointDeepLearningClassIDs.resize(writeIdx);
            if (hasClusterIDs) currentPointCloud->pointClusterIDs.resize(writeIdx);
            if (hasMarks) currentPointCloud->marks.resize(writeIdx);

            cachedPointCloud = currentPointCloud;

            this->parameter.needToRebuildSpatialPartitioning = true;

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

    class OperatorCompareWithLastPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCompareWithLastPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process(PointCloud* currentPointCloud) override
		{
			TS(CompareWithLastPointCloud);

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
                    if (comparisonDistanceThreshold < glm::distance(cp->positions[i], lp->positions[i]))
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
		virtual void Visualize() override
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
                auto& n = glm::normalize(cp->normals[i]);
                auto& c = cp->colors[i];

                if (cp->numberOfElements > lp->numberOfElements)
                {
                    if (matchedFlags[i])
                    {
                        VD::AddSphere("ComparisonResult_Matched", p, n, Configuration::pointVisualizationRadius, glm::vec4(c, 1.0f));
                    }
                    else
                    {
                        VD::AddSphere("ComparisonResult_Unmatched", p, n, Configuration::pointVisualizationRadius, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
                    }
                }
                else
                {
                    if (i < cp->numberOfElements)
                    {
                        if (matchedFlags[i])
                        {
                            VD::AddSphere("ComparisonResult_Matched", p, n, Configuration::pointVisualizationRadius, glm::vec4(c, 1.0f));
                        }
                        else
                        {
                            VD::AddSphere("ComparisonResult_Unmatched", p, n, Configuration::pointVisualizationRadius, glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));
                        }
                    }
                    else
                    {
                        auto& p2 = lp->positions[i];
                        auto& n2 = glm::normalize(lp->normals[i]);
                        auto& c2 = lp->colors[i];
						VD::AddSphere("ComparisonResult_Unmatched", p2, n2, Configuration::pointVisualizationRadius, glm::vec4(0.0f, 0.0f, 1.0f, 1.0f));
                    }
                }
			}
		}

		inline float GetComparisonDistanceThreshold() const { return comparisonDistanceThreshold; }
		inline void SetComparisonDistanceThreshold(float threshold) { comparisonDistanceThreshold = threshold; }

	private:
		float comparisonDistanceThreshold = 0.0001f;
		std::vector<bool> matchedFlags;
	};


    class OperatorClustering : public IGeometricProcessingOperator<SparseGrid>
    {
    public:
        OperatorClustering(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
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

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(Clustering_Parallel);

            if (currentPointCloud->numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
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

            std::for_each(std::execution::par, flatCells.begin(), flatCells.end(), [&](const CellData& cell)
                {
                    uint64_t key = cell.key;
                    int headIdx = cell.headIdx;

                    int gz = (int)(key & mask);
                    int gy = (int)((key >> 21) & mask);
                    int gx = (int)(key >> 42);

                    for (int i = headIdx; i != -1; i = spatialPartitioning->nextPoint[i])
                    {
                        const glm::vec3& pA = currentPointCloud->positions[i];
                        const glm::vec3& nA = currentPointCloud->normals[i];

                        for (int j = spatialPartitioning->nextPoint[i]; j != -1; j = spatialPartitioning->nextPoint[j])
                        {
                            const glm::vec3& pB = currentPointCloud->positions[j];

                            if (glm::distance2(pA, pB) > searchRadiusSq) continue;

                            const glm::vec3& nB = currentPointCloud->normals[j];

                            if (glm::dot(nA, nB) < strictAngleThreshold) continue;

                            float planeDist = std::abs(glm::dot(nA, pB - pA));
                            if (planeDist > planeDistThreshold) continue;

                            if (useMarksForClustering)
                            {
                                if (currentPointCloud->marks[i] != currentPointCloud->marks[j]) continue;
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
                                        const glm::vec3& pB = currentPointCloud->positions[j];

                                        if (glm::distance2(pA, pB) > searchRadiusSq) continue;

                                        const glm::vec3& nB = currentPointCloud->normals[j];

                                        if (glm::dot(nA, nB) < strictAngleThreshold) continue;

                                        float planeDist = std::abs(glm::dot(nA, pB - pA));
                                        if (planeDist > planeDistThreshold) continue;

                                        if (useMarksForClustering)
                                        {
                                            if (currentPointCloud->marks[i] != currentPointCloud->marks[j]) continue;
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
        OperatorClusterBorderFinding(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(ClusterBorderFinding);
            if (currentPointCloud->numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
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
                        const glm::vec3& pA = currentPointCloud->positions[i];
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
                                        const glm::vec3& pB = currentPointCloud->positions[j];
                                        if (glm::distance2(pA, pB) > searchRadiusSq) continue;
                                        int clusterB = currentPointCloud->pointClusterIDs[j];
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
        OperatorClusteringComplex(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
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

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(ComplexClustering);

            if (currentPointCloud->numberOfElements == 0) return;
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = currentPointCloud;
            size_t numPoints = currentPointCloud->numberOfElements;
            pointClusterIds.assign(numPoints, -1);
            pointCurvatures.resize(numPoints);

            // DL Class ID 데이터 유효성 확인
            bool hasValidClassIDs = params.useDeepLearningClasses &&
                !currentPointCloud->pointDeepLearningClassIDs.empty() &&
                (currentPointCloud->pointDeepLearningClassIDs.size() == numPoints);

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
                        glm::vec3 center = currentPointCloud->positions[i];
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
                                        if (glm::distance2(center, currentPointCloud->positions[idx]) <= curvSearchRadSq) {
                                            float dot = glm::dot(currentPointCloud->normals[i], currentPointCloud->normals[idx]);
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
                        const glm::vec3& pA = currentPointCloud->positions[i];
                        const glm::vec3& nA = currentPointCloud->normals[i];
                        const glm::vec3& cA = currentPointCloud->colors[i];
                        float curvA = pointCurvatures[i];
                        int classA = hasValidClassIDs ? currentPointCloud->pointDeepLearningClassIDs[i] : -1;

                        auto CheckAndMerge = [&](int j)
                            {
                                // 0. Class ID Check (Hard Constraint) [NEW]
                                // 서로 다른 클래스(예: 치아 vs 잇몸)라면 기하학적으로 가까워도 무조건 분리
                                if (hasValidClassIDs)
                                {
                                    if (classA != currentPointCloud->pointDeepLearningClassIDs[j]) return;
                                }

                                const glm::vec3& pB = currentPointCloud->positions[j];

                                // 1. Distance Check
                                if (glm::distance2(pA, pB) > searchRadiusSq) return;

                                const glm::vec3& nB = currentPointCloud->normals[j];

                                // 2. Normal Direction
                                if (glm::dot(nA, nB) < params.angleThreshold) return;

                                // 3. Plane Offset
                                if (std::abs(glm::dot(nA, pB - pA)) > planeDistAbs) return;

                                // 4. Color Check
                                if (glm::distance(cA, currentPointCloud->colors[j]) > params.colorThreshold) return;

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
        OperatorCurvatureEstimation(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(Curvature_Parallel);

            if (currentPointCloud->numberOfElements == 0) return;

            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = currentPointCloud;
            size_t numberOfPoints = currentPointCloud->numberOfElements;
            curvatures.resize(numberOfPoints);

            currentPointCloud->marks.clear();
			currentPointCloud->marks.resize(numberOfPoints, 0); // Reset Marks

            float searchRadius = spatialPartitioning->cellSize * searchRadiusScale;
            float searchRadiusSq = searchRadius * searchRadius;

            std::vector<int> indices(numberOfPoints);
            std::iota(indices.begin(), indices.end(), 0);

            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    const glm::vec3& p = currentPointCloud->positions[i];

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
                                        if (glm::distance2(p, currentPointCloud->positions[currIdx]) <= searchRadiusSq)
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

                    // Covariance Matrix (3x3 Symmetric)
                    // Cov = Sum( (p - c) * (p - c)^T )
                    float xx = 0, xy = 0, xz = 0;
                    float yy = 0, yz = 0, zz = 0;

                    for (int idx : neighbors)
                    {
                        glm::vec3 r = currentPointCloud->positions[idx] - centroid;
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
						currentPointCloud->marks[i] = 1; // High Curvature Mark
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
        int neighborSearchOffset = 3;
        float searchRadiusScale = 5.0f;
        float visualScale = 5.0f;

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
        OperatorCurvatureDivergence(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
		}

        std::vector<float> curvatures;
        std::vector<glm::vec3> gradients;
        std::vector<float> divergences;

        float searchRadiusMultiplier = 2.0f;
        float visualizationScale = 500.0f;

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(CurvatureDivergence_Total);

            if (currentPointCloud->numberOfElements == 0) return;

            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }
            
            cachedPointCloud = currentPointCloud;
            size_t numPoints = currentPointCloud->numberOfElements;

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
                    const glm::vec3& p = currentPointCloud->positions[i];
                    std::vector<int> neighbors;
                    neighbors.reserve(32);
                    glm::vec3 centroid(0.0f);

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
                        glm::vec3 r = currentPointCloud->positions[idx] - centroid;
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
                    const glm::vec3& p = currentPointCloud->positions[i];
                    float c_i = curvatures[i];
                    glm::vec3 gradSum(0.0f);
                    float weightSum = 0.0f;

                    ProcessNeighbors(i, p, *spatialPartitioning, searchRadiusSq, [&](int j) {
                        glm::vec3 diff = currentPointCloud->positions[j] - p;
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
                    const glm::vec3& p = currentPointCloud->positions[i];
                    const glm::vec3& g_i = gradients[i];
                    float divSum = 0.0f;
                    float count = 0.0f;

                    ProcessNeighbors(i, p, *spatialPartitioning, searchRadiusSq, [&](int j) {
                        glm::vec3 diff = currentPointCloud->positions[j] - p;
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
        OperatorMeshGeneration(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
        {
        }

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(MeshGeneration);

            if (currentPointCloud->numberOfElements == 0) return;

            // 1. Build Spatial Partitioning if needed (used for AABB mainly here)
            if (nullptr == spatialPartitioning)
            {
                spatialPartitioning = new SparseGrid();
                spatialPartitioning->Build(*currentPointCloud,Configuration::voxelSize);
                parameter.needToDeleteSpatialPartitioning = true;
            }

            cachedPointCloud = currentPointCloud;

            // 2. Initialize SparseDataBlock
            // Clear previous data to ensure clean generation
            sparseDataBlock.dataBlocks.clear();
            sparseDataBlock.voxelSize = meshVoxelSize;

            // 3. Populate Voxel Data (Implicit Surface Generation)
            // Converts explicit point cloud into implicit signed distance field
            sparseDataBlock.FromPointsData(
                currentPointCloud->positions,
                currentPointCloud->normals,
                currentPointCloud->colors,
                currentPointCloud->pointClusterIDs,
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
        OperatorMeshDistanceFilter(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
            : IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
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

        virtual void Process(PointCloud* currentPointCloud) override
        {
            TS(MeshDistanceFilter);

            if (currentPointCloud->numberOfElements == 0 || referenceMesh.empty()) return;

            cachedPointCloud = currentPointCloud;
            size_t numPoints = currentPointCloud->numberOfElements;

            distances.resize(numPoints);
            currentPointCloud->marks.assign(numPoints, 0);

            // 1. Build Spatial Grid for Triangles
            BuildTriangleGrid();

            std::vector<int> indices(numPoints);
            std::iota(indices.begin(), indices.end(), 0);

            // 2. Compute Distances (Parallel)
            std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
                {
                    float d = GetClosestDistanceFromMesh(currentPointCloud->positions[i]);

                    if (std::isnan(d) || std::isinf(d)) d = FLT_MAX;

                    distances[i] = d;
                });

            int markedCount = 0;
            for (size_t i = 0; i < numPoints; ++i)
            {
                //if (distances[i] > averageDistance * thresholdMultiplier)
                //{
                //    currentPointCloud->marks[i] = 1;
                //    markedCount++;
                //}

                if (distances[i] > Configuration::voxelSize * thresholdMultiplier)
                {
                    currentPointCloud->marks[i] = 1;
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

                op->Process(currentPointCloud, sparseGrid);

                auto operatorMeshGeneration = std::dynamic_pointer_cast<OperatorMeshGeneration>(op);
                if (operatorMeshGeneration)
                {
                    generatedMeshTriangles = operatorMeshGeneration->GetTriangles();
                }

                std::cout << Miliseconds(time, tag.c_str()) << std::endl;
            }

            {
                auto time = std::chrono::high_resolution_clock::now();

                if (op->GetParameter().needToRebuildSpatialPartitioning)
                {
                    BuildSparseGrid(*currentPointCloud);
                }

                std::cout << Miliseconds(time, "Rebuilding Spatial Grid") << std::endl;
            }
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

    void Pipeline::CreatePointCloud()
    {
		pointClouds.emplace_back(PointCloud());
		currentPointCloud = &pointClouds.back();
    }

    void Pipeline::StorePointCloud()
    {
        pointClouds.push_back(currentPointCloud->Clone());
        currentPointCloud = &pointClouds.back();
    }

    void Pipeline::RestoreInitialPointCloud()
    {
        if (!pointClouds.empty())
        {
            currentPointCloud = &pointClouds.front();
		}
    }

    void Pipeline::RestoreLastPointCloud()
    {
        if (!pointClouds.empty())
        {
            currentPointCloud = &pointClouds.back();
        }
    }

#pragma endregion
}
