#include <iostream>
#include <vector>
#include <algorithm>
#include <omp.h> // OpenMP

#include <libFeather.h>

using VD = VisualDebugging;
using namespace libRxTx;

// --- Helper Functions ---
static inline void DepthToColor(float d, unsigned char& r, unsigned char& g, unsigned char& b)
{
	d = std::clamp(d, 0.0f, 1.0f);
	float r_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.75f)), 0.0f, 1.0f);
	float g_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.50f)), 0.0f, 1.0f);
	float b_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.25f)), 0.0f, 1.0f);
	r = static_cast<unsigned char>(r_f * 255.0f);
	g = static_cast<unsigned char>(g_f * 255.0f);
	b = static_cast<unsigned char>(b_f * 255.0f);
}

int main(int argc, char** argv)
{
	std::cout << "AppFeather" << std::endl;

	Feather.Initialize(1920, 1080);
	Feather.SetConsoleWindowIndex(3);
	Feather.SetMainWindowIndex(2);

	auto w = Feather.GetFeatherWindow();

#pragma region AppMain
	{
		auto appMain = Feather.CreateEntity("AppMain");
		Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
			if (GLFW_KEY_ESCAPE == event.keyCode)
			{
				glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
			}
			else if (GLFW_KEY_SPACE == event.keyCode)
			{
				if (event.action == 0)
				{
					Feather.GetImmediateModeRenderSystem()->ToggleEnable();
				}
			}
			else if (GLFW_KEY_ENTER == event.keyCode && event.action == 0)
			{
				glBindFramebuffer(GL_FRAMEBUFFER, 0);

				auto w = Feather.GetFeatherWindow();

				int width, height;
				glfwGetFramebufferSize(w->GetGLFWwindow(), &width, &height);

				std::vector<float> depth(width * height);
				glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data());

				// Flip Y
				std::vector<float> flipped(width * height);
				for (int y = 0; y < height; ++y)
				{
					memcpy(&flipped[y * width],
						&depth[(height - 1 - y) * width],
						width * sizeof(float));
				}

				// 깊이값 최소/최대 구하기
				float minD = 1.0f, maxD = 0.0f;
				for (float v : flipped)
				{
					if (v < minD) minD = v;
					if (v > maxD) maxD = v;
				}
				printf("[Depth] min=%.6f, max=%.6f, range=%.6f\n", minD, maxD, maxD - minD);
				float range = std::max(maxD - minD, 1e-6f);

				// RAW 저장
				FILE* f = fopen("depth.raw", "wb");
				fwrite(flipped.data(), sizeof(float), width * height, f);
				fclose(f);

				// pseudo-color PNG
				std::vector<unsigned char> rgb(width * height * 3);
				for (int i = 0; i < width * height; ++i)
				{
					float norm = (flipped[i] - minD) / range;
					unsigned char r, g, b;
					DepthToColor(norm, r, g, b);
					rgb[i * 3 + 0] = r;
					rgb[i * 3 + 1] = g;
					rgb[i * 3 + 2] = b;
				}

				stbi_write_png("depth_color.png", width, height, 3, rgb.data(), width * 3);
				printf("[DepthMap] Saved to depth_color.png\n");
			}
			});
	}
#pragma endregion

	Feather.AddOnInitializeCallback([&]() {

#pragma region Camera
		{
			Entity cam = Feather.CreateEntity("Camera");
			auto pcam = Feather.CreateComponent<PerspectiveCamera>(cam);
			auto pcamMan = Feather.CreateComponent<CameraManipulatorTrackball>(cam);
			pcamMan->SetCamera(pcam);

			Feather.CreateEventCallback<FrameBufferResizeEvent>(cam, [pcam](Entity entity, const FrameBufferResizeEvent& event) {
				auto window = Feather.GetFeatherWindow();
				auto aspectRatio = (f32)window->GetWidth() / (f32)window->GetHeight();
				pcam->SetAspectRatio(aspectRatio);
				});

			Feather.CreateEventCallback<KeyEvent>(cam, [](Entity entity, const KeyEvent& event) {
				Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnKey(event);
				});

			Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity entity, const MousePositionEvent& event) {
				Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMousePosition(event);
				});

			Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity entity, const MouseButtonEvent& event) {
				auto manipulator = Feather.GetComponent<CameraManipulatorTrackball>(entity);
				manipulator->OnMouseButton(event);

				if (event.button == 0 && event.action == 0)
				{
					int fbWidth, fbHeight;
					glfwGetFramebufferSize(w->GetGLFWwindow(), &fbWidth, &fbHeight);

					int readX = static_cast<int>(event.xpos);
					int readY = fbHeight - static_cast<int>(event.ypos) - 1;

					float depth = 0.0f;
					glReadPixels(readX, readY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
					printf("Depth: %f\n", depth);
				}
				});

			Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) {
				Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event);
				});
		}
#pragma endregion

#pragma region Status Panel
		{
			auto gui = Feather.GetRegistry().create();
			auto statusPanel = Feather.GetRegistry().emplace<StatusPanel>(gui);

			Feather.CreateEventCallback<MousePositionEvent>(gui, [](Entity entity, const MousePositionEvent& event) {
				auto& component = Feather.GetRegistry().get<StatusPanel>(entity);
				component.mouseX = event.xpos;
				component.mouseY = event.ypos;
				});
		}
#pragma endregion

		// ---------------------------------------------------------
		// Point Cloud & TSDF Processing (Dense Grid Version)
		// ---------------------------------------------------------
		{
			struct Point
			{
				glm::vec3 position;
				glm::vec3 normal;
				glm::vec3 color;
			};

			std::vector<Point> points;

			// 1. Load Point Cloud
			PLYFormat ply;
			ply.Deserialize("D:\\Debug\\PLY\\input.ply");

			AABB aabb{ {FLT_MAX, FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX, -FLT_MAX} };
			auto lmin = glm::vec3(-10.0f, -10.0f, -10.0f);
			auto lmax = glm::vec3(10.0f, 10.0f, 10.0f);

			for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
			{
				auto x = ply.GetPoints()[3 * i + 0];
				auto y = ply.GetPoints()[3 * i + 1];
				auto z = ply.GetPoints()[3 * i + 2];
				glm::vec3 position = glm::vec3(x, y, z);

				if (position.x < lmin.x || position.y < lmin.y || position.z < lmin.z) continue;
				if (position.x > lmax.x || position.y > lmax.y || position.z > lmax.z) continue;

				aabb.min.x = fminf(aabb.min.x, x);
				aabb.min.y = fminf(aabb.min.y, y);
				aabb.min.z = fminf(aabb.min.z, z);

				aabb.max.x = fmaxf(aabb.max.x, x);
				aabb.max.y = fmaxf(aabb.max.y, y);
				aabb.max.z = fmaxf(aabb.max.z, z);

				auto nx = ply.GetNormals()[3 * i + 0];
				auto ny = ply.GetNormals()[3 * i + 1];
				auto nz = ply.GetNormals()[3 * i + 2];
				glm::vec3 normal = glm::vec3(nx, ny, nz);

				glm::vec3 color;
				if (ply.UseAlpha())
				{
					color.r = ply.GetColors()[4 * i + 0];
					color.g = ply.GetColors()[4 * i + 1];
					color.b = ply.GetColors()[4 * i + 2];
				}
				else
				{
					color.r = ply.GetColors()[3 * i + 0];
					color.g = ply.GetColors()[3 * i + 1];
					color.b = ply.GetColors()[3 * i + 2];
				}
				points.push_back({ position, normal, color });
			}

			TS(Total);

			// 2. Build Dense TSDF Grid
			// -----------------------------------------------------
			struct TSDFVoxel
			{
				float distSum;
				float weightSum;
				glm::vec3 normalSum;
				glm::vec3 colorSum;
			};

			TS(Build_TSDF_Volume_Grid);

			float voxelSize = 0.2f; // Grid 해상도
			float trunc = 0.6f;     // Truncation distance
			int influenceRadius = (int)std::ceil(trunc / voxelSize);

			glm::vec3 size = aabb.max - aabb.min;
			int Nx = (int)(size.x / voxelSize) + 3;
			int Ny = (int)(size.y / voxelSize) + 3;
			int Nz = (int)(size.z / voxelSize) + 3;

			// Dense Grid 할당 (0으로 초기화됨)
			std::vector<TSDFVoxel> voxelGrid(Nx * Ny * Nz, { 0.0f, 0.0f, glm::vec3(0), glm::vec3(0) });

			// TSDF 통합 (Integration)
			// Race Condition 방지를 위해 Grid 쓰기는 Serial로 수행 (배열 접근이라 빠름)
			for (const auto& pt : points)
			{
				// 포인트의 그리드 좌표
				int pix = (int)std::floor((pt.position.x - aabb.min.x) / voxelSize);
				int piy = (int)std::floor((pt.position.y - aabb.min.y) / voxelSize);
				int piz = (int)std::floor((pt.position.z - aabb.min.z) / voxelSize);

				// 주변 복셀 순회 및 업데이트
				for (int z = -influenceRadius; z <= influenceRadius; ++z)
				{
					int iz = piz + z;
					if (iz < 0 || iz >= Nz) continue;

					for (int y = -influenceRadius; y <= influenceRadius; ++y)
					{
						int iy = piy + y;
						if (iy < 0 || iy >= Ny) continue;

						for (int x = -influenceRadius; x <= influenceRadius; ++x)
						{
							int ix = pix + x;
							if (ix < 0 || ix >= Nx) continue;

							// 1D Index 계산 (O(1))
							int idx = (iz * Ny + iy) * Nx + ix;

							// 복셀 중심 위치
							glm::vec3 voxelCenter = aabb.min + glm::vec3(ix + 0.5f, iy + 0.5f, iz + 0.5f) * voxelSize;

							glm::vec3 diff = voxelCenter - pt.position;
							float distRaw = glm::length(diff);

							if (distRaw > trunc) continue;

							float sign = glm::dot(diff, pt.normal) < 0.0f ? -1.0f : 1.0f;
							float sdf = sign * distRaw;

							// Grid 업데이트
							auto& v = voxelGrid[idx];
							float weight = 1.0f;

							v.distSum += sdf * weight;
							v.weightSum += weight;
							v.normalSum += pt.normal * weight;
							v.colorSum += pt.color * weight;
						}
					}
				}
			}

			printf("Grid Size = %d x %d x %d (%zu voxels)\n", Nx, Ny, Nz, voxelGrid.size());

			// TSDF 정규화 (병렬 처리 가능)
#pragma omp parallel for
			for (int i = 0; i < (int)voxelGrid.size(); ++i)
			{
				auto& v = voxelGrid[i];
				if (v.weightSum > 0.0f)
				{
					v.distSum /= v.weightSum;
					v.normalSum = glm::normalize(v.normalSum);
					v.colorSum /= v.weightSum;
					v.distSum = std::clamp(v.distSum, -trunc, trunc);
				}
				else
				{
					// 데이터가 없는 빈 공간은 Trunc Distance(1.0 or trunc)로 초기화
					v.distSum = 1.0f;
				}
			}
			TE(Build_TSDF_Volume_Grid);


			// 3. Marching Cubes (Dense Grid + Parallel)
			// -----------------------------------------------------
			struct MCVertex
			{
				glm::vec3 pos;
				glm::vec3 norm;
				glm::vec3 col;
			};

			struct MCMesh
			{
				std::vector<MCVertex> vertices;
				std::vector<uint32_t> indices;
			};

			MCMesh mesh;
			float isoValue = 0.0f;

			TS(Build_Mesh_Grid);

			// 스레드별 버퍼
			struct ThreadBuffer {
				std::vector<MCVertex> vertices;
				std::vector<uint32_t> indices;
			};
			int maxThreads = omp_get_max_threads();
			std::vector<ThreadBuffer> threadBuffers(maxThreads);

			// OpenMP Grid Loop
			// 3중 루프를 collapse하여 하나의 큰 루프로 병렬화
#pragma omp parallel for collapse(3) schedule(static)
			for (int z = 0; z < Nz - 1; ++z)
			{
				for (int y = 0; y < Ny - 1; ++y)
				{
					for (int x = 0; x < Nx - 1; ++x)
					{
						int tid = omp_get_thread_num();

						// 큐브의 8개 코너에 대한 Grid Index 계산
						int idx[8];
						idx[0] = (z * Ny + y) * Nx + x;             // (x,   y,   z)
						idx[1] = idx[0] + 1;                        // (x+1, y,   z)
						idx[2] = idx[0] + Nx + 1;                   // (x+1, y+1, z)
						idx[3] = idx[0] + Nx;                       // (x,   y+1, z)
						idx[4] = idx[0] + Nx * Ny;                  // (x,   y,   z+1)
						idx[5] = idx[4] + 1;                        // (x+1, y,   z+1)
						idx[6] = idx[4] + Nx + 1;                   // (x+1, y+1, z+1)
						idx[7] = idx[4] + Nx;                       // (x,   y+1, z+1)

						float cubeVal[8];
						glm::vec3 cubePos[8];
						glm::vec3 cubeNorm[8];
						glm::vec3 cubeCol[8];

						int cornerOffsets[8][3] = {
							{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
							{0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}
						};

						// 데이터 Fetch (배열 접근이라 매우 빠름)
						for (int j = 0; j < 8; ++j)
						{
							const auto& v = voxelGrid[idx[j]];
							cubeVal[j] = v.distSum;
							cubeNorm[j] = v.normalSum;
							cubeCol[j] = v.colorSum;

							cubePos[j] = aabb.min + glm::vec3(x + cornerOffsets[j][0], y + cornerOffsets[j][1], z + cornerOffsets[j][2]) * voxelSize;
						}

						// Marching Cubes Logic
						int cubeIndex = 0;
						if (cubeVal[0] < isoValue) cubeIndex |= 1;
						if (cubeVal[1] < isoValue) cubeIndex |= 2;
						if (cubeVal[2] < isoValue) cubeIndex |= 4;
						if (cubeVal[3] < isoValue) cubeIndex |= 8;
						if (cubeVal[4] < isoValue) cubeIndex |= 16;
						if (cubeVal[5] < isoValue) cubeIndex |= 32;
						if (cubeVal[6] < isoValue) cubeIndex |= 64;
						if (cubeVal[7] < isoValue) cubeIndex |= 128;

						if (edgeTable[cubeIndex] == 0) continue;

						glm::vec3 vP[12], vN[12], vC[12];

						auto InterpL = [&](const glm::vec3& p1, const glm::vec3& p2, float v1, float v2) {
							return p1 + (isoValue - v1) / (v2 - v1) * (p2 - p1);
							};

						if (edgeTable[cubeIndex] & 1) {
							vP[0] = InterpL(cubePos[0], cubePos[1], cubeVal[0], cubeVal[1]);
							vN[0] = InterpL(cubeNorm[0], cubeNorm[1], cubeVal[0], cubeVal[1]);
							vC[0] = InterpL(cubeCol[0], cubeCol[1], cubeVal[0], cubeVal[1]);
						}
						if (edgeTable[cubeIndex] & 2) {
							vP[1] = InterpL(cubePos[1], cubePos[2], cubeVal[1], cubeVal[2]);
							vN[1] = InterpL(cubeNorm[1], cubeNorm[2], cubeVal[1], cubeVal[2]);
							vC[1] = InterpL(cubeCol[1], cubeCol[2], cubeVal[1], cubeVal[2]);
						}
						if (edgeTable[cubeIndex] & 4) {
							vP[2] = InterpL(cubePos[2], cubePos[3], cubeVal[2], cubeVal[3]);
							vN[2] = InterpL(cubeNorm[2], cubeNorm[3], cubeVal[2], cubeVal[3]);
							vC[2] = InterpL(cubeCol[2], cubeCol[3], cubeVal[2], cubeVal[3]);
						}
						if (edgeTable[cubeIndex] & 8) {
							vP[3] = InterpL(cubePos[3], cubePos[0], cubeVal[3], cubeVal[0]);
							vN[3] = InterpL(cubeNorm[3], cubeNorm[0], cubeVal[3], cubeVal[0]);
							vC[3] = InterpL(cubeCol[3], cubeCol[0], cubeVal[3], cubeVal[0]);
						}
						if (edgeTable[cubeIndex] & 16) {
							vP[4] = InterpL(cubePos[4], cubePos[5], cubeVal[4], cubeVal[5]);
							vN[4] = InterpL(cubeNorm[4], cubeNorm[5], cubeVal[4], cubeVal[5]);
							vC[4] = InterpL(cubeCol[4], cubeCol[5], cubeVal[4], cubeVal[5]);
						}
						if (edgeTable[cubeIndex] & 32) {
							vP[5] = InterpL(cubePos[5], cubePos[6], cubeVal[5], cubeVal[6]);
							vN[5] = InterpL(cubeNorm[5], cubeNorm[6], cubeVal[5], cubeVal[6]);
							vC[5] = InterpL(cubeCol[5], cubeCol[6], cubeVal[5], cubeVal[6]);
						}
						if (edgeTable[cubeIndex] & 64) {
							vP[6] = InterpL(cubePos[6], cubePos[7], cubeVal[6], cubeVal[7]);
							vN[6] = InterpL(cubeNorm[6], cubeNorm[7], cubeVal[6], cubeVal[7]);
							vC[6] = InterpL(cubeCol[6], cubeCol[7], cubeVal[6], cubeVal[7]);
						}
						if (edgeTable[cubeIndex] & 128) {
							vP[7] = InterpL(cubePos[7], cubePos[4], cubeVal[7], cubeVal[4]);
							vN[7] = InterpL(cubeNorm[7], cubeNorm[4], cubeVal[7], cubeVal[4]);
							vC[7] = InterpL(cubeCol[7], cubeCol[4], cubeVal[7], cubeVal[4]);
						}
						if (edgeTable[cubeIndex] & 256) {
							vP[8] = InterpL(cubePos[0], cubePos[4], cubeVal[0], cubeVal[4]);
							vN[8] = InterpL(cubeNorm[0], cubeNorm[4], cubeVal[0], cubeVal[4]);
							vC[8] = InterpL(cubeCol[0], cubeCol[4], cubeVal[0], cubeVal[4]);
						}
						if (edgeTable[cubeIndex] & 512) {
							vP[9] = InterpL(cubePos[1], cubePos[5], cubeVal[1], cubeVal[5]);
							vN[9] = InterpL(cubeNorm[1], cubeNorm[5], cubeVal[1], cubeVal[5]);
							vC[9] = InterpL(cubeCol[1], cubeCol[5], cubeVal[1], cubeVal[5]);
						}
						if (edgeTable[cubeIndex] & 1024) {
							vP[10] = InterpL(cubePos[2], cubePos[6], cubeVal[2], cubeVal[6]);
							vN[10] = InterpL(cubeNorm[2], cubeNorm[6], cubeVal[2], cubeVal[6]);
							vC[10] = InterpL(cubeCol[2], cubeCol[6], cubeVal[2], cubeVal[6]);
						}
						if (edgeTable[cubeIndex] & 2048) {
							vP[11] = InterpL(cubePos[3], cubePos[7], cubeVal[3], cubeVal[7]);
							vN[11] = InterpL(cubeNorm[3], cubeNorm[7], cubeVal[3], cubeVal[7]);
							vC[11] = InterpL(cubeCol[3], cubeCol[7], cubeVal[3], cubeVal[7]);
						}

						for (int k = 0; triTable[cubeIndex][k] != -1; k += 3)
						{
							int a = triTable[cubeIndex][k + 0];
							int b = triTable[cubeIndex][k + 1];
							int c = triTable[cubeIndex][k + 2];

							// [핵심 수정] 1. 생성될 삼각형의 기하학적 노멀 계산
							glm::vec3 triNormal = glm::cross(vP[b] - vP[a], vP[c] - vP[a]);

							// [핵심 수정] 2. 복셀에 저장된 원본 포인트들의 평균 노멀 가져오기
							glm::vec3 storedNormal(0.0f);
							int validNormals = 0;
							for (int j = 0; j < 8; ++j) {
								// 데이터가 존재했던 복셀의 노멀만 합산 (weightSum > 0 등 체크하면 더 좋음)
								if (glm::length(cubeNorm[j]) > 0.01f) {
									storedNormal += cubeNorm[j];
									validNormals++;
								}
							}

							// [핵심 수정] 3. 방향 비교 (Back-Face Removal)
							// 저장된 노멀이 없으면(validNormals==0) 일단 그림 (혹은 제거 선택)
							if (validNormals > 0) {
								// 삼각형 노멀과 원본 노멀이 반대 방향(음수)이면 "뒷면"이므로 생성 스킵
								if (glm::dot(triNormal, storedNormal) < 0.0f) {
									continue;
								}
							}

							// --- 통과된 삼각형만 버퍼에 추가 ---
							auto& buf = threadBuffers[tid];
							uint32_t bi = (uint32_t)buf.vertices.size();

							buf.vertices.push_back({ vP[a], glm::normalize(vN[a]), vC[a] });
							buf.vertices.push_back({ vP[b], glm::normalize(vN[b]), vC[b] });
							buf.vertices.push_back({ vP[c], glm::normalize(vN[c]), vC[c] });

							buf.indices.push_back(bi + 0);
							buf.indices.push_back(bi + 1);
							buf.indices.push_back(bi + 2);
						}
					}
				}
			}

			// Merge Thread Buffers
			size_t totalVertices = 0;
			size_t totalIndices = 0;
			for (const auto& buf : threadBuffers) {
				totalVertices += buf.vertices.size();
				totalIndices += buf.indices.size();
			}

			mesh.vertices.reserve(totalVertices);
			mesh.indices.reserve(totalIndices);

			uint32_t indexOffset = 0;
			for (const auto& buf : threadBuffers)
			{
				mesh.vertices.insert(mesh.vertices.end(), buf.vertices.begin(), buf.vertices.end());
				for (uint32_t idx : buf.indices) {
					mesh.indices.push_back(idx + indexOffset);
				}
				indexOffset += (uint32_t)buf.vertices.size();
			}

			TE(Build_Mesh_Grid);


			// ====================================================
			// 0. Vertex Welding (정점 병합)
			// Half-Edge를 만들려면 정점이 공유되어야 합니다.
			// ====================================================
			std::vector<MCVertex> weldedVertices;
			std::vector<uint32_t> weldedIndices;

			// Vec3 비교를 위한 엡실론
			auto isSame = [](const glm::vec3& a, const glm::vec3& b) {
				return glm::length(a - b) < 0.0001f;
				};

			// 위치 해싱을 위해 정수를 키로 사용 (간단한 공간 해싱)
			struct Vec3Hash {
				size_t operator()(const glm::vec3& v) const {
					return std::hash<float>()(v.x) ^ std::hash<float>()(v.y) ^ std::hash<float>()(v.z);
				}
			};

			// 병합을 위한 맵 (Key: Position, Value: New Index)
			// 편의상 느린 검색 대신 정밀도를 위해 간단히 구현 (실제론 Octree나 Grid Hash 권장)
			// 여기서는 성능을 위해 정밀도 손실을 감수하고 map 사용
			std::map<std::tuple<int, int, int>, uint32_t> vMap;
			float scale = 10000.0f; // 소수점 4자리까지 구분

			for (size_t i = 0; i < mesh.vertices.size(); ++i)
			{
				const auto& v = mesh.vertices[i];
				std::tuple<int, int, int> key = {
					(int)(v.pos.x * scale),
					(int)(v.pos.y * scale),
					(int)(v.pos.z * scale)
				};

				auto it = vMap.find(key);
				if (it != vMap.end())
				{
					weldedIndices.push_back(it->second);
				}
				else
				{
					uint32_t newIdx = (uint32_t)weldedVertices.size();
					vMap[key] = newIdx;
					weldedVertices.push_back(v);
					weldedIndices.push_back(newIdx);
				}
			}

			printf("Welding: %zu -> %zu vertices\n", mesh.vertices.size(), weldedVertices.size());


			// ====================================================
			// 1. Half-Edge Data Structure Definition
			// ====================================================
			struct HalfEdge {
				uint32_t targetVertex; // 이 엣지가 가리키는 정점 인덱스
				int twinEdge = -1;     // 짝꿍 엣지 인덱스 (-1이면 Border)
				// int nextEdge;       // Loop 순회용 (지금은 Border 추출만 하므로 생략 가능)
			};

			// (StartVertex, EndVertex) -> EdgeIndex 맵
			// 키를 uint64_t로 패킹: (Start << 32) | End
			std::unordered_map<uint64_t, int> edgeMap;
			std::vector<HalfEdge> halfEdges;

			// ====================================================
			// 2. Build Half-Edge Mesh
			// ====================================================
			for (size_t i = 0; i < weldedIndices.size(); i += 3)
			{
				uint32_t idx[3] = { weldedIndices[i], weldedIndices[i + 1], weldedIndices[i + 2] };

				for (int j = 0; j < 3; ++j)
				{
					uint32_t u = idx[j];
					uint32_t v = idx[(j + 1) % 3];

					// 현재 엣지 생성 (u -> v)
					int currentEdgeIdx = (int)halfEdges.size();
					halfEdges.push_back({ v, -1 }); // twin은 아직 모름

					// 맵에 등록
					uint64_t key = ((uint64_t)u << 32) | v;
					edgeMap[key] = currentEdgeIdx;

					// Twin 찾기 (v -> u 가 이미 존재하는지 확인)
					uint64_t twinKey = ((uint64_t)v << 32) | u;
					auto it = edgeMap.find(twinKey);
					if (it != edgeMap.end())
					{
						int twinEdgeIdx = it->second;
						// 서로 연결
						halfEdges[currentEdgeIdx].twinEdge = twinEdgeIdx;
						halfEdges[twinEdgeIdx].twinEdge = currentEdgeIdx;
					}
				}
			}

			// ====================================================
			// 3. Extract Borders
			// Twin이 없는 엣지가 바로 Border입니다.
			// ====================================================
			std::vector<glm::vec3> borderLines;
			std::vector<glm::vec3> borderColors; // 시각화용

			for (auto const& [key, edgeIdx] : edgeMap)
			{
				const auto& he = halfEdges[edgeIdx];

				if (he.twinEdge == -1) // Twin이 없다 == 경계선이다
				{
					uint32_t u = (uint32_t)(key >> 32);
					uint32_t v = (uint32_t)(key & 0xFFFFFFFF);

					borderLines.push_back(weldedVertices[u].pos);
					borderLines.push_back(weldedVertices[v].pos);

					// 빨간색으로 표시
					borderColors.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
					borderColors.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
				}
			}

			printf("Found %zu border edges.\n", borderLines.size() / 2);

			TE(Total);

			// ====================================================
			// 4. Visualize Borders (Create a Lines Entity)
			// ====================================================
			if (!borderLines.empty())
			{
				auto borderEntity = Feather.CreateEntity("BorderLines");
				auto lineRenderable = Feather.CreateComponent<Renderable>(borderEntity);

				// GL_LINES 모드로 설정
				lineRenderable->Initialize(Renderable::GeometryMode::Lines);

				lineRenderable->AddShader(
					Feather.CreateShader(
						"BorderShader",
						File("../../res/Shaders/Default.vs"),
						File("../../res/Shaders/Default.fs")
					)
				);
				lineRenderable->SetActiveShaderIndex(0);

				lineRenderable->AddVertices(borderLines);
				lineRenderable->AddColors(borderColors);

				// 인덱스는 순서대로 0, 1, 2, 3...
				std::vector<uint32_t> lineIndices(borderLines.size());
				std::iota(lineIndices.begin(), lineIndices.end(), 0);
				lineRenderable->AddIndices(lineIndices);
			}



			// 4. Create Renderable
			auto meshEntity = Feather.CreateEntity("TSDFMesh");
			auto meshRenderable = Feather.CreateComponent<Renderable>(meshEntity);
			meshRenderable->Initialize(Renderable::GeometryMode::Triangles);

			meshRenderable->AddShader(
				Feather.CreateShader(
					"TSDFMeshShader",
					File("../../res/Shaders/Default.vs"),
					File("../../res/Shaders/Default.fs")
				)
			);
			meshRenderable->SetActiveShaderIndex(0);

			std::vector<glm::vec3> outVerts;
			std::vector<glm::vec3> outNorms;
			std::vector<glm::vec3> outCols;
			std::vector<uint32_t> outIdx;

			outVerts.reserve(mesh.vertices.size());
			outNorms.reserve(mesh.vertices.size());
			outCols.reserve(mesh.vertices.size());

			for (auto& v : mesh.vertices)
			{
				outVerts.push_back(v.pos);
				outNorms.push_back(v.norm);
				outCols.push_back(v.col);
			}

			outIdx = mesh.indices;

			meshRenderable->AddVertices(outVerts);
			meshRenderable->AddNormals(outNorms);
			meshRenderable->AddColors(outCols);
			meshRenderable->AddIndices(outIdx);

			// Key Events for Rendering Modes
			Feather.CreateEventCallback<KeyEvent>(meshEntity, [](Entity entity, const KeyEvent& event) {
				auto renderable = Feather.GetComponent<Renderable>(entity);
				if (nullptr == renderable) return;

				if (0 == event.action)
				{
					if (GLFW_KEY_GRAVE_ACCENT == event.keyCode)
					{
						renderable->NextDrawingMode();
					}
					else if (GLFW_KEY_1 == event.keyCode)
					{
						renderable->SetActiveShaderIndex(0);
					}
					else if (GLFW_KEY_2 == event.keyCode)
					{
						renderable->SetActiveShaderIndex(1);
					}
				}
				});
		}
		});

	Feather.Run();
	Feather.Terminate();

	return 0;
}
