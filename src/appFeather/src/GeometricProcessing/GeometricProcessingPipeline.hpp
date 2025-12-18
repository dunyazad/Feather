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

		virtual void Process() override
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
					Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f)
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

		virtual void Process() override
		{
			TS(StorePointCloud);
			pipeline->StorePointCloud();
			TE(StorePointCloud);
		}

		virtual void Visualize() override
		{
		}
	};

	class OperatorRestoreInitialPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorRestoreInitialPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
		{
			TS(StorePointCloud);
			pipeline->RestoreInitialPointCloud();
			TE(StorePointCloud);
		}

		virtual void Visualize() override
		{
		}
	};

	class OperatorRestoreLastPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorRestoreLastPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
		{
			TS(StorePointCloud);
			pipeline->RestoreLastPointCloud();
			TE(StorePointCloud);
		}

		virtual void Visualize() override
		{
		}
	};

	class OperatorPointCloudVisualization : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointCloudVisualization(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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
					Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f)
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

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;

			float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
			float searchRadiusSq = searchRadius * searchRadius;

			std::vector<int> indices(numPoints);
			std::iota(indices.begin(), indices.end(), 0);

			std::vector<Eigen::Vector3f> nextPositions = currentPointCloud->positions;

			for (int iter = 0; iter < iterations; ++iter)
			{
				std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
					{
						if (preserveMarks && !currentPointCloud->marks.empty() && currentPointCloud->marks[i] != 0)
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

		virtual void Visualize() override
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

		inline void SetIterations(int iter) { iterations = iter; }
		inline int GetIterations() const { return iterations; }

		inline void SetSmoothingFactor(float lambda) { smoothingFactor = std::clamp(lambda, 0.0f, 1.0f); }
		inline float GetSmoothingFactor() const { return smoothingFactor; }

		inline float GetSearchRadiusMultiplier() const { return searchRadiusMultiplier; }
		inline void SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; }

		inline int GetNeighborSearchOffset() const { return neighborSearchOffset; }
		inline void SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; }

		inline bool IsPreserveMarks() const { return preserveMarks; }
		inline void SetPreserveMarks(bool preserve) { preserveMarks = preserve; }

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
		OperatorKNNSmoothing(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;

			std::vector<int> indices(numPoints);
			std::iota(indices.begin(), indices.end(), 0);

			std::vector<Eigen::Vector3f> nextPositions = currentPointCloud->positions;

			for (int iter = 0; iter < iterations; ++iter)
			{
				std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
					{
						if (false == (preserveMarks && !currentPointCloud->marks.empty() && currentPointCloud->marks[i] != 0))
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
					Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f)
				);
			}
		}

		inline void SetK(int k) { kNeighbors = k; }
		inline int GetK() const { return kNeighbors; }

		inline void SetIterations(int iter) { iterations = iter; }
		inline int GetIterations() const { return iterations; }

		inline void SetSmoothingFactor(float factor) { smoothingFactor = std::clamp(factor, 0.0f, 1.0f); }
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

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;

			for (int i = 0; i < iteration; i++)
			{
				std::vector<Eigen::Vector3f> newPositions = currentPointCloud->positions;
				std::vector<Eigen::Vector3f> newNormals = currentPointCloud->normals;

				std::vector<int> indices(numPoints);
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
					Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f)
				);
			}
		}

		inline void SetKNeighbors(int k) { kNeighbors = k; }
		inline void SetUpdateNormals(bool update) { updateNormals = update; }
		inline void SetPreserveMarks(bool preserve) { preserveMarks = preserve; }

	private:
		int kNeighbors = 32;
		int iteration = 10;
		bool updateNormals = true;
		bool preserveMarks = true;

		void ComputeEigenDecomposition(const Eigen::Matrix3f& cov, Eigen::Vector3f& outEvals, Eigen::Matrix3f& outEvecs)
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
	};

	class OperatorPointDensity : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorPointDensity(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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

		virtual void Visualize() override
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

		inline float GetSearchRadiusScale() const { return searchRadiusScale; }
		inline void SetSearchRadiusScale(float scale) { searchRadiusScale = scale; }

		inline int GetNeighborSearchOffset() const { return neighborSearchOffset; }
		inline void SetNeighborSearchOffset(int offset) { neighborSearchOffset = offset; }

		inline float GetMinDensity() const { return minDensity; }
		inline float GetMaxDensity() const { return maxDensity; }

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
		OperatorFindOverlappingPoints(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;

			// -------------------------------------------------------------
			// [파라미터 로드] 무조건 각도(Degree)로 받습니다.
			// -------------------------------------------------------------
			{
				overlapDistanceThreshold = parameter.GetParameter<float>("overlapDistanceThreshold", overlapDistanceThreshold);

				// "몇 도까지 봐줄거냐?" (기본값: 15도)
				maxNormalAngle = parameter.GetParameter<float>("maxNormalAngle", maxNormalAngle);
			}

			// [내부 변환] 성능을 위해 루프 돌기 전에 딱 한 번만 변환합니다.
			// 각도(Degree) -> 라디안 -> 코사인 값
			// 예: 15도 -> 0.9659
			float internalCosThreshold = std::cos(maxNormalAngle * 3.14159265f / 180.0f);

			currentPointCloud->marks.assign(numPoints, 0);
			overlappingCount = 0;

			float thresholdSq = overlapDistanceThreshold * overlapDistanceThreshold;

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

										// 중복 제거: 인덱스가 큰 쪽을 제거 대상으로 마킹
										if (curr < other)
										{
											const Eigen::Vector3f& pB = currentPointCloud->positions[other];

											if ((pA - pB).squaredNorm() <= thresholdSq)
											{
												const Eigen::Vector3f& nB = currentPointCloud->normals[other];

												// [비교 로직]
												// 내적(Dot) 결과는 두 벡터가 벌어질수록 작아집니다. (0도=1.0, 90도=0.0)
												// 따라서 "내적값 >= 코사인임계값" 이어야 "각도 <= 설정각도"가 성립합니다.
												if (nA.dot(nB) >= internalCosThreshold)
												{
													currentPointCloud->marks[other] = 1;
												}
											}
										}
									}
								}
							}
						}
					}
				});

			for (int m : currentPointCloud->marks) {
				if (m == 1) overlappingCount++;
			}

			TE(FindOverlappingPoints);
		}

		virtual void Visualize() override
		{
			if (nullptr == cachedPointCloud) return;

			size_t count = cachedPointCloud->numberOfElements;

			for (size_t i = 0; i < count; ++i)
			{
				if (cachedPointCloud->marks[i] == 1)
				{
					VD::AddSphere(
						"OverlappingPoints",
						cachedPointCloud->positions[i],
						Configuration::pointVisualizationRadius * 1.2f,
						Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f) // Red (중복)
					);
				}
				else
				{
					VD::AddSphere(
						"UniquePoints",
						cachedPointCloud->positions[i],
						Configuration::pointVisualizationRadius,
						Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.1f) // Blue (생존)
					);
				}
			}
		}

		// 사용자는 이제 '각도'만 신경 쓰면 됩니다.
		inline void SetMaxNormalAngle(float degrees) { maxNormalAngle = degrees; }
		inline float GetMaxNormalAngle() const { return maxNormalAngle; }

		inline void SetOverlapDistanceThreshold(float threshold) { overlapDistanceThreshold = threshold; }
		inline int GetOverlappingCount() const { return overlappingCount; }

	private:
		float overlapDistanceThreshold = 1e-4f;
		float maxNormalAngle = 15.0f; // 사용자가 설정하는 각도 (기본 15도)

		int overlappingCount = 0;
	};

#pragma region Morphological
	class MorphologyHelper
	{
	public:
		// 1. 모폴로지 연산 (위치 이동)
		// mode: 0 = Erosion (Min), 1 = Dilation (Max)
		static void ApplyMorphology(
			std::shared_ptr<PointCloud> cloud,
			SparseGrid* grid,
			float radius,
			int mode,
			int neighborSearchOffset = 1)
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
											// 투영 높이 계산
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

		// 법선(Normal) 방향 기준으로 '위쪽(튀어나온 쪽)'에 있는 포인트를 제거 대상으로 마킹
		static void MarkOverlappingPointsAbove(
			std::shared_ptr<PointCloud> cloud,
			SparseGrid* grid,
			float overlapDistThreshold = 1e-4f,
			float maxAngle = 15.0f)
		{
			// 성능을 위해 Grid Rebuild는 외부에서 수행되었다고 가정합니다.

			float thresholdSq = overlapDistThreshold * overlapDistThreshold;
			// 각도 -> Cos 변환
			float minCos = std::cos(maxAngle * 3.14159265f / 180.0f);

			// 마킹 초기화
			cloud->marks.assign(cloud->numberOfElements, 0);

			// [수정] 병렬 루프 내에서는 오직 '마킹(Marking)'만 수행합니다. (positions.push_back 삭제)
			// 서로 다른 인덱스에 접근하므로 Thread-safe 합니다.
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

										// 중복 검사는 한 번만 수행하기 위해 pair (A, B) 중 인덱스 작은 쪽에서만 로직 수행
										if (curr < other)
										{
											// 1. 거리 체크
											Eigen::Vector3f diff = cloud->positions[other] - pA;
											if (diff.squaredNorm() <= thresholdSq)
											{
												// 2. 법선 방향 체크 (비슷한 방향을 보고 있어야 함)
												if (nA.dot(cloud->normals[other]) >= minCos)
												{
													// 3. 누가 더 위에 있는가? (높이 판별)
													// B가 A의 법선 방향으로 얼마나 떨어져 있는지 내적 계산
													float heightDiff = diff.dot(nA);

													if (heightDiff > 0)
													{
														// B가 A보다 위에 있음 (Normal 방향) -> B 삭제
														cloud->marks[other] = 1;
													}
													else
													{
														// A가 B보다 위에 있음 (Normal 반대 방향에 B가 있음) -> A 삭제
														cloud->marks[curr] = 1;
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

			// [수정] 병렬 처리가 끝난 후, 단일 스레드에서 안전하게 데이터를 수집하여 저장합니다.
			PLYFormat ply;
			size_t count = cloud->numberOfElements;
			for (size_t i = 0; i < count; i++)
			{
				if (cloud->marks[i] == 1)
				{
					const auto& p = cloud->positions[i];
					if (FLT_MAX == p.x() || FLT_MAX == p.y() || FLT_MAX == p.z())
					{
						continue;
					}
					ply.AddPointFloat3(p.data());
				}
			}

			ply.Serialize("D:\\Temp\\PLY\\Morphology_OverlappingPoints.ply");
		}
	};

	class OperatorApplyMorphology : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorApplyMorphology(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
		{
			TS(ApplyMorphology);

			auto currentPointCloud = pipeline->GetCurrentPointCloud();
			if (currentPointCloud->numberOfElements == 0) return;

			// 1. 초기 Grid 빌드 (현재 위치 기준)
			if (nullptr == spatialPartitioning) {
				spatialPartitioning = new SparseGrid();
				spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
				parameter.needToDeleteSpatialPartitioning = true;
			}
			cachedPointCloud = currentPointCloud;

			// 2. 파라미터 로드
			{
				operation = parameter.GetParameter<std::string>("operation", "Erosion"); // 기본값 Erosion
				radius = parameter.GetParameter<float>("radius", radius);
				overlapDist = parameter.GetParameter<float>("overlapDist", overlapDist);
				maxAngle = parameter.GetParameter<float>("maxAngle", maxAngle);
				// [추가] 마킹만 하고 원본 위치로 복원할지 여부
				markingOnly = parameter.GetParameter<bool>("markingOnly", markingOnly);
			}

			// [추가] 원본 위치 백업 (Marking Only 모드일 경우)
			std::vector<Eigen::Vector3f> originalPositions;
			if (markingOnly)
			{
				originalPositions = currentPointCloud->positions;
			}

			// 3. Operation 분기 처리 (여기서 점들의 위치가 이동합니다)
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

			// 4. Grid 재구축 (필수)
			// 점들의 위치가 변했으므로, 정확한 중복 체크를 위해 Grid를 최신화합니다.
			if (spatialPartitioning != nullptr)
			{
				spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
			}

			// 5. 중복 점 마킹 (변형된 위치 기준으로 Overlap 계산 및 마킹)
			MorphologyHelper::MarkOverlappingPointsAbove(currentPointCloud, spatialPartitioning, overlapDist, maxAngle);

			// [추가] 원본 위치 복원 (Marking Only 모드일 경우)
			if (markingOnly)
			{
				// 위치를 원본으로 되돌립니다. (마킹 정보는 그대로 유지됨)
				currentPointCloud->positions = originalPositions;

				// 위치가 다시 바뀌었으므로 Grid도 원본 기준으로 다시 맞춰줍니다. (안전장치)
				if (spatialPartitioning != nullptr)
				{
					spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
				}
			}

			TE(ApplyMorphology);
		}

		virtual void Visualize() override
		{
			if (nullptr == cachedPointCloud) return;

			size_t count = cachedPointCloud->numberOfElements;
			for (size_t i = 0; i < count; ++i)
			{
				// markingOnly가 true였다면, 여기서 시각화되는 위치는 원본 위치입니다.
				if (cachedPointCloud->marks[i] == 1)
				{
					// 중복되어 지워질 점: Red
					VD::AddSphere(
						"Morphology_Marked_Removed",
						cachedPointCloud->positions[i],
						Configuration::pointVisualizationRadius * 1.2f,
						Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f)
					);
				}
				else
				{
					// 살아남은 점: Blue (반투명)
					VD::AddSphere(
						"Morphology_Marked_Kept",
						cachedPointCloud->positions[i],
						Configuration::pointVisualizationRadius,
						Eigen::Vector4f(0.0f, 0.0f, 1.0f, 0.2f)
					);
				}
			}
		}

	private:
		std::string operation = "Erosion";
		float radius = 0.05f;
		float overlapDist = 1e-4f;
		float maxAngle = 15.0f;
		bool markingOnly = false; // [추가] 기본값 false
	};
#pragma endregion

#pragma region About Normal
	class OperatorNormalDeviation : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalDeviation(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;

			{ // Apply parameters
				searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
				neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
				maxDeviationAngle = parameter.GetParameter<float>("maxDeviationAngle", maxDeviationAngle);
			}

			deviations.resize(numPoints);
			currentPointCloud->marks.assign(numPoints, 0);

			float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
			float searchRadiusSq = searchRadius * searchRadius;

			std::vector<int> indices(numPoints);
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
											// 법선 방향이 반대인 경우 보정하여 합산
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
						currentPointCloud->marks[i] = 1;
					}
				});

			TE(NormalDeviation);
		}

		virtual void Visualize() override
		{
			VisualizeDefault();
		}

		void VisualizeDefault()
		{
			if (nullptr == cachedPointCloud || deviations.empty()) return;

			size_t count = cachedPointCloud->numberOfElements;

			for (size_t i = 0; i < count; ++i)
			{
				float val = deviations[i];

				// Normal points: Blue to Cyan based on small deviation
				// Outlier points (Marked): Red

				if (cachedPointCloud->marks[i] == 1)
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
					// Visualize degree of deviation for normal points
					// Map 0 ~ maxDeviationAngle to Blue ~ Cyan
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

		void VisualizeHeatmap()
		{
			if (nullptr == cachedPointCloud || deviations.empty()) return;

			size_t count = cachedPointCloud->numberOfElements;

			// 히트맵 시각화: 0도(Blue) -> maxDeviationAngle(Red)
			for (size_t i = 0; i < count; ++i)
			{
				float val = deviations[i];

				// 0.0 ~ 1.0 정규화 (최대 각도 기준)
				float t = std::clamp(val / maxDeviationAngle, 0.0f, 1.0f);

				Eigen::Vector3f color;

				// Blue(0.0) -> Cyan -> Green(0.5) -> Yellow -> Red(1.0) 히트맵 로직
				if (t < 0.25f)
				{
					// Blue -> Cyan
					float localT = t / 0.25f;
					color = Eigen::Vector3f(0.0f, localT, 1.0f);
				}
				else if (t < 0.5f)
				{
					// Cyan -> Green
					float localT = (t - 0.25f) / 0.25f;
					color = Eigen::Vector3f(0.0f, 1.0f, 1.0f - localT);
				}
				else if (t < 0.75f)
				{
					// Green -> Yellow
					float localT = (t - 0.5f) / 0.25f;
					color = Eigen::Vector3f(localT, 1.0f, 0.0f);
				}
				else
				{
					// Yellow -> Red
					float localT = (t - 0.75f) / 0.25f;
					color = Eigen::Vector3f(1.0f, 1.0f - localT, 0.0f);
				}

				VD::AddSphere(
					"NormalDeviationHeatmap",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius, // 크기는 일정하게
					Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
				);
			}
		}

		inline void SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; }
		inline void SetMaxDeviationAngle(float angle) { maxDeviationAngle = angle; }

		inline const std::vector<float>& GetDeviations() const { return deviations; }

	private:
		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;
		float maxDeviationAngle = 30.0f; // 이 각도가 넘으면 Red, 0도면 Blue

		std::vector<float> deviations;
	};

	class OperatorNormalGradient : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalGradient(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;

			// 파라미터 로드
			{
				searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
				neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
				useAlphaGradient = parameter.GetParameter<bool>("useAlphaGradient", useAlphaGradient);
				visualizationSigma = parameter.GetParameter<float>("visualizationSigma", visualizationSigma);
			}

			gradients.assign(numPoints, 0.0f);

			float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
			float searchRadiusSq = searchRadius * searchRadius;

			std::vector<int> indices(numPoints);
			std::iota(indices.begin(), indices.end(), 0);

			// -----------------------------------------------------------
			// [Calculation] 법선 변화율(Gradient) 계산
			// -----------------------------------------------------------
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

											// 법선 차이 계산 (Cosine Distance)
											// 0.0 (Parallel) ~ 1.0 (Orthogonal) ~ 2.0 (Opposite)
											// 내적값이 1에 가까울수록 차이가 없으므로 (1 - dot)을 사용
											float dot = n_i.dot(currentPointCloud->normals[curr]);

											// 법선 방향이 뒤집힌 경우(Back-face)를 같은 면으로 볼지 여부에 따라 abs() 사용 결정
											// 여기서는 기하학적 '꺾임'을 보므로 방향성 고려하여 clamp만 적용
											float normalDiff = 1.0f - std::clamp(dot, -1.0f, 1.0f);

											float weight = 1.0f / dist; // 거리가 가까울수록 가중치 높음

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

			// -----------------------------------------------------------
			// [Statistics] 시각화 자동 조정을 위한 통계 계산
			// -----------------------------------------------------------
			double sum = 0.0;
			double sqSum = 0.0;
			for (float v : gradients)
			{
				sum += v;
				sqSum += v * v;
			}
			gradientMean = (float)(sum / numPoints);
			double variance = (sqSum / numPoints) - (gradientMean * gradientMean);
			gradientStdDev = (float)std::sqrt(std::max(0.0, variance));

			TE(NormalGradient);
		}

		virtual void Visualize() override
		{
			if (nullptr == cachedPointCloud || gradients.empty()) return;

			size_t count = cachedPointCloud->numberOfElements;

			// Auto-Exposure Threshold Calculation
			// 평균 + (표준편차 * Sigma)를 최대값(Red)으로 설정
			float maxThreshold = gradientMean + gradientStdDev * visualizationSigma;

			// 최소값은 0(변화 없음)에 가깝지만, 노이즈 제거를 위해 평균 이하를 Blue로 밉니다.
			float minThreshold = gradientMean * 0.5f;

			float range = maxThreshold - minThreshold;
			if (range < 1e-6f) range = 1.0f;

			for (size_t i = 0; i < count; ++i)
			{
				float val = gradients[i];

				// 값 클램핑 (Outlier 제거 효과)
				float clampedVal = std::clamp(val, minThreshold, maxThreshold);

				// 정규화 0.0 ~ 1.0
				float t = (clampedVal - minThreshold) / range;

				Eigen::Vector3f color;

				// Heatmap: Blue(Flat) -> Cyan -> Green -> Yellow -> Red(Sharp Edge)
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

				VD::AddSphere(
					"NormalGradientMap",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(color.x(), color.y(), color.z(), useAlphaGradient ? t : 1.0f)
				);
			}
		}

		inline void SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; }
		inline void SetVisualizationSigma(float sigma) { visualizationSigma = sigma; }

		// 분석용 Getter
		inline float GetGradientMean() const { return gradientMean; }
		inline float GetGradientStdDev() const { return gradientStdDev; }

	private:
		std::vector<float> gradients;

		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;
		bool useAlphaGradient = false;

		// 시각화용 통계 변수 (2.0 ~ 3.0 추천)
		float visualizationSigma = 3.0f;
		float gradientMean = 0.0f;
		float gradientStdDev = 0.0f;
	};

	class OperatorNormalDivergence : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalDivergence(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;

			{ // Apply parameter
				searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
				neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
				visualizationScale = parameter.GetParameter<float>("visualizationScale", visualizationScale); // 이 변수는 Heatmap 방식에선 안 쓰일 수도 있으나 유지
			}

			normalDivergences.assign(numPoints, 0.0f);

			float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
			float searchRadiusSq = searchRadius * searchRadius;

			std::vector<int> indices(numPoints);
			std::iota(indices.begin(), indices.end(), 0);

			// 1. 계산 (병렬 처리)
			// 주의: 여기서 minDivergence, maxDivergence를 건드리면 Race Condition 발생
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

			// 2. Min/Max 계산 (안전하게 후처리)
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

		virtual void Visualize() override
		{
			VisualizeHeatmap();
		}

		void VisualizeHeatmap()
		{
			if (nullptr == cachedPointCloud || normalDivergences.empty()) return;

			size_t count = cachedPointCloud->numberOfElements;

			// 1. 평균(Mean)과 표준편차(StdDev) 계산
			//    (Visualize는 자주 호출되지 않으므로 여기서 계산해도 성능상 무방합니다)
			double sum = 0.0;
			double sqSum = 0.0;

			// 노이즈가 아닌 유효 데이터만 추리기 위해, 
			// 너무 큰 값(예: FLT_MAX)은 제외하고 계산할 수도 있지만, 
			// 여기서는 전체를 다 계산해도 표준편차 덕분에 보정이 됩니다.
			for (float v : normalDivergences)
			{
				sum += v;
				sqSum += v * v;
			}

			double mean = sum / count;
			double variance = (sqSum / count) - (mean * mean);
			double stdDev = std::sqrt(std::max(0.0, variance));

			// 2. Limit 설정 (핵심!)
			// 표준편차의 N배를 Limit으로 설정합니다.
			// 2.0f (2-Sigma): 전체 데이터의 약 95%를 포함 (대비가 강함, 추천)
			// 3.0f (3-Sigma): 전체 데이터의 약 99%를 포함 (대비가 조금 약함)
			// visualizationScale을 여기서 'Sigma 배수'로 활용하면 좋습니다. (예: 기본 2.0)
			float sigmaMultiplier = 2.0f;
			// 만약 외부 파라미터 visualizationScale을 쓴다면:
			// float sigmaMultiplier = (visualizationScale > 0.1f) ? visualizationScale : 2.0f;

			float limit = (float)(stdDev * sigmaMultiplier);

			// 만약 데이터가 너무 평평해서 limit이 0에 가까우면 강제로 키움 (Divide by Zero 방지)
			if (limit < 1e-6f) limit = 1.0f;

			// 시각화 범위: -limit ~ +limit
			// 0(평평함) = Green, -limit(오목) = Blue, +limit(볼록) = Red

			for (size_t i = 0; i < count; ++i)
			{
				float val = normalDivergences[i];

				// 3. 값 자르기 (Clamping)
				// 노이즈(limit 밖의 값)를 limit으로 고정
				float clampedVal = std::clamp(val, -limit, limit);

				// 4. 정규화 (-limit ~ +limit  ->  0.0 ~ 1.0)
				float t = (clampedVal + limit) / (2.0f * limit);

				Eigen::Vector3f color;

				// [Color Map]
				// 0.0 (Min Limit) -> Blue
				// 0.5 (Zero/Flat) -> Green
				// 1.0 (Max Limit) -> Red
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

				VD::AddSphere(
					"NormalDivergenceHeatmap",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
				);
			}
		}

		inline void SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; }
		inline void SetVisualizationScale(float scale) { visualizationScale = scale; }

	private:
		std::vector<float> normalDivergences;
		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;
		float visualizationScale = 50.0f;
		float minDivergence = 0.0f;
		float maxDivergence = 0.0f;
	};

	class OperatorNormalDivergenceGradient : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorNormalDivergenceGradient(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
		{
			TS(NormalDivergenceGradient);

			auto currentPointCloud = pipeline->GetCurrentPointCloud();
			if (currentPointCloud->numberOfElements == 0) return;

			// 1. 공간 분할 구조 빌드
			if (nullptr == spatialPartitioning)
			{
				spatialPartitioning = new SparseGrid();
				spatialPartitioning->Build(*currentPointCloud, Configuration::voxelSize);
				parameter.needToDeleteSpatialPartitioning = true;
			}

			cachedPointCloud = currentPointCloud;
			size_t numPoints = currentPointCloud->numberOfElements;

			// 2. 파라미터 로드
			{
				searchRadiusMultiplier = parameter.GetParameter<float>("searchRadiusMultiplier", searchRadiusMultiplier);
				neighborSearchOffset = parameter.GetParameter<int>("neighborSearchOffset", neighborSearchOffset);
				// 시각화 시 표준편차의 몇 배수 이상을 강조할지 결정 (기본 2.0 = 상위 5% 내외의 급격한 변화)
				visualizationSigma = parameter.GetParameter<float>("visualizationSigma", visualizationSigma);
			}

			// 3. 메모리 할당
			normalDivergences.assign(numPoints, 0.0f);
			divergenceGradients.assign(numPoints, 0.0f);

			float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
			float searchRadiusSq = searchRadius * searchRadius;

			std::vector<int> indices(numPoints);
			std::iota(indices.begin(), indices.end(), 0);

			// ----------------------------------------------------------------
			// [Pass 1] Normal Divergence 계산 (곡률/요철 계산)
			// ----------------------------------------------------------------
			std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
				{
					const Eigen::Vector3f& p = currentPointCloud->positions[i];
					const Eigen::Vector3f& n_i = currentPointCloud->normals[i];

					float divSum = 0.0f;
					float weightSum = 0.0f;

					// 이웃 검색 람다 (코드 중복 방지용 구조)
					auto ProcessNeighbors = [&](int neighborIdx)
						{
							Eigen::Vector3f diff = currentPointCloud->positions[neighborIdx] - p;
							float distSq = diff.squaredNorm();

							if (distSq <= searchRadiusSq && distSq > 1e-8f)
							{
								float dist = std::sqrt(distSq);
								Eigen::Vector3f dir = diff / dist;
								Eigen::Vector3f n_diff = currentPointCloud->normals[neighborIdx] - n_i;

								// Divergence: 법선 차이와 방향 벡터의 내적
								float dotVal = n_diff.dot(dir);
								float weight = 1.0f / dist;

								divSum += dotVal * weight;
								weightSum += weight;
							}
						};

					// Grid 탐색
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

			// ----------------------------------------------------------------
			// [Pass 2] Gradient 계산 (Divergence의 변화량 계산)
			// ----------------------------------------------------------------
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
											// Gradient: 이웃과의 Divergence 차이 절댓값 평균
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

			// 4. 통계 계산 (시각화용 Mean, StdDev)
			double sum = 0.0;
			double sqSum = 0.0;
			for (float v : divergenceGradients)
			{
				sum += v;
				sqSum += v * v;
			}
			gradientMean = (float)(sum / numPoints);
			float variance = (float)((sqSum / numPoints) - (gradientMean * gradientMean));
			gradientStdDev = std::sqrt(std::max(0.0f, variance));

			TE(NormalDivergenceGradient);
		}

		virtual void Visualize() override
		{
			if (nullptr == cachedPointCloud || divergenceGradients.empty()) return;

			size_t count = cachedPointCloud->numberOfElements;

			// Auto-Thresholding: 평균 + (표준편차 * Sigma)
			// 이 값보다 작은 변화는 "평평함(Smooth)"으로 간주
			float thresholdLow = gradientMean + gradientStdDev * 0.5f;   // 여기서부터 색상 시작
			float thresholdHigh = gradientMean + gradientStdDev * visualizationSigma; // 여기가 최대값(Red)

			float range = thresholdHigh - thresholdLow;
			if (range < 1e-6f) range = 1.0f;

			for (size_t i = 0; i < count; ++i)
			{
				float val = divergenceGradients[i];

				// Threshold 미만은 무시 (또는 아주 연하게) -> Noise Filtering 효과
				if (val < thresholdLow)
				{
					// 변화가 거의 없는 곳: 투명한 회색 or 원래 점 색상
					VD::AddSphere(
						"Gradient_Stable",
						cachedPointCloud->positions[i],
						Configuration::pointVisualizationRadius * 0.8f, // 조금 작게
						Eigen::Vector4f(0.5f, 0.5f, 0.5f, 0.1f) // 아주 연한 회색
					);
					continue;
				}

				// 정규화 (0.0 ~ 1.0)
				float t = std::clamp((val - thresholdLow) / range, 0.0f, 1.0f);

				Eigen::Vector3f color;

				// Gradient Map: Blue(약한 변화) -> Green -> Yellow -> Red(급격한 변화)
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

				// 급격한 변화일수록 점을 약간 키워서 강조
				float radiusScale = 1.0f + t * 0.5f;

				VD::AddSphere(
					"Gradient_RapidChange",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius * radiusScale,
					Eigen::Vector4f(color.x(), color.y(), color.z(), 1.0f)
				);
			}
		}

		inline void SetSearchRadiusMultiplier(float mult) { searchRadiusMultiplier = mult; }
		inline void SetVisualizationSigma(float sigma) { visualizationSigma = sigma; }

		// 분석용 Getter
		inline float GetGradientMean() const { return gradientMean; }
		inline float GetGradientStdDev() const { return gradientStdDev; }

	private:
		std::vector<float> normalDivergences; // Pass 1 결과
		std::vector<float> divergenceGradients; // Pass 2 결과 (최종 변화량)

		float searchRadiusMultiplier = 2.0f;
		int neighborSearchOffset = 1;

		// 시각화 제어용 통계 변수
		float visualizationSigma = 3.0f; // 표준편차의 3배 이상을 가장 빨갛게 표시 (조절 가능)
		float gradientMean = 0.0f;
		float gradientStdDev = 0.0f;
	};
#pragma endregion

	template<typename FilterFunctor>
	class OperatorCustomFilter : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCustomFilter(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
		{
			TS(CustomFilter);

			auto currentPointCloud = pipeline->GetCurrentPointCloud();
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
				auto& n = cachedPointCloud->normals[i].normalized();
				auto& c = cachedPointCloud->colors[i];
				VD::AddSphere("FilteredPoints", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f));
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

		virtual void Process() override
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
				auto& n = cachedPointCloud->normals[i].normalized();
				auto& c = cachedPointCloud->colors[i];
				VD::AddSphere("FilteredPoints", p, n, Configuration::pointVisualizationRadius, Eigen::Vector4f(c.x(), c.y(), c.z(), 1.0f));
			}
		}
	};

	class OperatorCompareSameIndexOrderedPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCompareSameIndexOrderedPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
		{
			TS(CompareOrderedPointCloud);

			int indexA = parameter.GetParameter<int>("indexA", 0);  // 원본 (삭제 전)
			int indexB = parameter.GetParameter<int>("indexB", -1); // 결과 (삭제 후)

			pointCloudA = pipeline->GetPointCloud(indexA);
			pointCloudB = pipeline->GetPointCloud(indexB);

			if (!pointCloudA || !pointCloudB)
			{
				TE(CompareOrderedPointCloud);
				return;
			}

			size_t countA = pointCloudA->numberOfElements;
			size_t countB = pointCloudB->numberOfElements;

			matchedFlags.assign(countA, 0); // 0: 삭제됨, 1: 생존함
			deletedCount = 0;

			// [Two-Pointer Algorithm]
			// A와 B를 동시에 훑습니다.
			size_t idxB = 0;
			for (size_t idxA = 0; idxA < countA; ++idxA)
			{
				bool isMatch = false;

				// B가 아직 끝까지 안 갔다면 비교 수행
				if (idxB < countB)
				{
					const Eigen::Vector3f& pA = pointCloudA->positions[idxA];
					const Eigen::Vector3f& pB = pointCloudB->positions[idxB];

					// 좌표가 같으면 (또는 오차 범위 이내면) 같은 점으로 인정
					// Morphology MarkingOnly 옵션을 썼다면 좌표가 완벽히 같아야 합니다.
					if ((pA - pB).squaredNorm() <= comparisonDistanceThresholdSq)
					{
						isMatch = true;
					}
				}

				if (isMatch)
				{
					// 매칭 성공: A의 이 점은 살아남아서 B의 idxB 위치에 있습니다.
					matchedFlags[idxA] = 1;

					// 둘 다 다음 칸으로 이동
					idxB++;
				}
				else
				{
					// 매칭 실패: A의 이 점은 B 목록에서 누락(삭제)되었습니다.
					matchedFlags[idxA] = 0;
					deletedCount++;

					// A만 다음 칸으로 이동 (idxB는 제자리에서 A의 다음 점을 기다림)
				}
			}

			alog("Ordered Compare Result: Total A(%zu), Total B(%zu), Deleted(%d)\n", countA, countB, deletedCount);

			cachedPointCloud = pointCloudA;
			TE(CompareOrderedPointCloud);
		}

		virtual void Visualize() override
		{
			if (!pointCloudA || matchedFlags.empty()) return;

			size_t count = pointCloudA->numberOfElements;
			PLYFormat ply;

			for (size_t i = 0; i < count; ++i)
			{
				const auto& p = pointCloudA->positions[i];
				const auto& n = pointCloudA->normals[i].normalized();

				if (matchedFlags[i])
				{
					// 살아남은 점 (초록색, 투명도 높게)
					VD::AddSphere(
						"OrderedCompare_Matched",
						p, n,
						Configuration::pointVisualizationRadius,
						Eigen::Vector4f(0.0f, 1.0f, 0.0f, 0.2f));
				}
				else
				{
					// 삭제된 점 (빨간색, 진하게)
					VD::AddSphere(
						"OrderedCompare_Deleted",
						p, n,
						Configuration::pointVisualizationRadius,
						Eigen::Vector4f(1.0f, 0.0f, 0.0f, 1.0f));

					if (FLT_MAX != p.x()) ply.AddPointFloat3(p.data());
				}
			}

			// 삭제된 점들만 PLY로 저장
			ply.Serialize("D:\\Debug\\PLY\\Compare_Ordered_Deleted.ply");
		}

		inline void SetComparisonThreshold(float dist) { comparisonDistanceThresholdSq = dist * dist; }

	private:
		std::vector<char> matchedFlags;
		std::shared_ptr<PointCloud> pointCloudA;
		std::shared_ptr<PointCloud> pointCloudB;
		float comparisonDistanceThresholdSq = 1e-5f; // 매우 작은 값 (좌표가 거의 같아야 함)
		int deletedCount = 0;
	};

	class OperatorComparePointCloudUsingDistance : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorComparePointCloudUsingDistance(Pipeline* pipeline,
			const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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

			// ----------------------------------------------------
			// 1. B 기준 SparseGrid 생성
			// ----------------------------------------------------
			SparseGrid gridB;
			gridB.Build(*pointCloudB, Configuration::voxelSize);

			size_t countA = pointCloudA->numberOfElements;
			matchedFlags.assign(countA, 0);

			float thresholdSq = comparisonDistanceThreshold * comparisonDistanceThreshold;

			// ----------------------------------------------------
			// 2. A의 각 포인트에 대해 B에서 근접 포인트 탐색
			// ----------------------------------------------------
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

		virtual void Visualize() override
		{
			if (!pointCloudA || matchedFlags.empty())
				return;

			size_t count = pointCloudA->numberOfElements;

			PLYFormat ply;

			for (size_t i = 0; i < count; ++i)
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

					if (FLT_MAX != p.x() && FLT_MAX != p.y() && FLT_MAX != p.z())
					{
						ply.AddPointFloat3(p.data());
					}
				}
			}

			ply.Serialize("D:\\Temp\\PLY\\ComparePointCloud.ply");
		}

		inline void SetComparisonDistanceThreshold(float t)
		{
			comparisonDistanceThreshold = t;
		}

	private:
		float comparisonDistanceThreshold = 0.0000001f;

		std::vector<char> matchedFlags;

		std::shared_ptr<PointCloud> pointCloudA;
		std::shared_ptr<PointCloud> pointCloudB;
	};

	class OperatorCompareWithLastPointCloud : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorCompareWithLastPointCloud(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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

		virtual void Process() override
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

		virtual void Process() override
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
				auto& n = cachedPointCloud->normals[i].normalized();
				auto& c = cachedPointCloud->colors[i];
				auto& mark = cachedPointCloud->marks[i];

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
			float searchRadiusMult = 1.5f;
			float angleThreshold = 0.9f;
			float planeOffsetThreshold = 0.2f;
			float colorThreshold = 0.15f;
			float curvatureDiffThreshold = 0.05f;
			bool useDeepLearningClasses = true;
		};

		ClusteringParams params;

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

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;
			pointClusterIds.assign(numPoints, -1);
			pointCurvatures.resize(numPoints);

			bool hasValidClassIDs = params.useDeepLearningClasses &&
				!currentPointCloud->pointDeepLearningClassIDs.empty() &&
				(currentPointCloud->pointDeepLearningClassIDs.size() == numPoints);

			TS(PrecalcFeatures);
			{
				std::vector<int> indices(numPoints);
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

				auto color = colors[id % 64];

				VD::AddSphere("ComplexClusters",
					cachedPointCloud->positions[i],
					Configuration::pointVisualizationRadius,
					Eigen::Vector4f(color.r, color.g, color.b, color.a));
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

		virtual void Process() override
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
			curvatures.resize(numberOfPoints);

			currentPointCloud->marks.clear();
			currentPointCloud->marks.resize(numberOfPoints, 0);

			float searchRadius = spatialPartitioning->cellSize * searchRadiusScale;
			float searchRadiusSq = searchRadius * searchRadius;

			std::vector<int> indices(numberOfPoints);
			std::iota(indices.begin(), indices.end(), 0);

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
						currentPointCloud->marks[i] = 1;
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

				Eigen::Vector4f color = Eigen::Vector4f(cachedPointCloud->colors[i].x(), cachedPointCloud->colors[i].y(), cachedPointCloud->colors[i].z(), 1.0f);
				if (cachedPointCloud->marks[i] == 1)
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
		OperatorCurvatureDivergence(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		std::vector<float> curvatures;
		std::vector<Eigen::Vector3f> gradients;
		std::vector<float> divergences;

		float searchRadiusMultiplier = 2.0f;
		float visualizationScale = 500.0f;

		virtual void Process() override
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
			size_t numPoints = currentPointCloud->numberOfElements;

			curvatures.assign(numPoints, 0.0f);
			gradients.assign(numPoints, Eigen::Vector3f::Zero());
			divergences.assign(numPoints, 0.0f);

			float searchRadius = spatialPartitioning->cellSize * searchRadiusMultiplier;
			float searchRadiusSq = searchRadius * searchRadius;

			std::vector<int> indices(numPoints);
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

	private:

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

	class OperatorMeshGeneration : public IGeometricProcessingOperator<SparseGrid>
	{
	public:
		OperatorMeshGeneration(Pipeline* pipeline, const GeometricProcessingOperatorParameter& parameter)
			: IGeometricProcessingOperator<SparseGrid>(pipeline, parameter)
		{
		}

		virtual void Process() override
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

		virtual void Visualize() override
		{
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

		virtual void Process() override
		{
			TS(MeshDistanceFilter);

			auto currentPointCloud = pipeline->GetCurrentPointCloud();
			if (currentPointCloud->numberOfElements == 0 || referenceMesh.empty()) return;

			cachedPointCloud = currentPointCloud;
			size_t numPoints = currentPointCloud->numberOfElements;

			distances.resize(numPoints);
			currentPointCloud->marks.assign(numPoints, 0);

			BuildTriangleGrid();

			std::vector<int> indices(numPoints);
			std::iota(indices.begin(), indices.end(), 0);

			std::for_each(std::execution::par, indices.begin(), indices.end(), [&](int i)
				{
					float d = GetClosestDistanceFromMesh(currentPointCloud->positions[i]);

					if (std::isnan(d) || std::isinf(d)) d = FLT_MAX;

					distances[i] = d;
				});

			int markedCount = 0;
			for (size_t i = 0; i < numPoints; ++i)
			{
				if (distances[i] > Configuration::voxelSize * thresholdMultiplier)
				{
					currentPointCloud->marks[i] = 1;
					markedCount++;
				}
			}

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
		Eigen::Vector3f triGridMin = Eigen::Vector3f::Zero();

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

		float GetClosestDistanceFromMesh(const Eigen::Vector3f& p)
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

		float SqDistPointTriangle(const Eigen::Vector3f& p, const Triangle& tri)
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
}
