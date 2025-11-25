#include <iostream>
#include <vector>
#include <algorithm>
#include <omp.h> // OpenMP
#include <map>
#include <tuple>
#include <numeric>

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

			if (0 == points.size())
			{
				printf("Error, points.size() is 0\n");
			}

			TS(Total);

			// -----------------------------------------------------
			// 2. Build Dense TSDF Grid (Fast Parallel Version)
			// -----------------------------------------------------
			struct TSDFVoxel
			{
				float distSum;
				float weightSum;
				glm::vec3 normalSum;
				glm::vec3 colorSum;
			};

			// Helper: OpenMP용 Atomic Float Add
			auto AtomicAddFloat = [](float& target, float value) {
#pragma omp atomic
				target += value;
				};

			TS(Build_TSDF_Volume_Grid);

			float voxelSize = 0.2f;
			float trunc = 0.4f;
			float truncSq = trunc * trunc; // 제곱 거리 비교용
			int influenceRadius = (int)std::ceil(trunc / voxelSize);

			glm::vec3 size = aabb.max - aabb.min;
			// 그리드 크기에 약간의 여유를 더 줌 (+5)
			int Nx = (int)(size.x / voxelSize) + 5;
			int Ny = (int)(size.y / voxelSize) + 5;
			int Nz = (int)(size.z / voxelSize) + 5;

			// 0.0f가 아닌 1.0f(외부)로 초기화하는 것이 Meshing때 깔끔함 (weight가 0일때 대비)
			std::vector<TSDFVoxel> voxelGrid(Nx * Ny * Nz, { 0.0f, 0.0f, glm::vec3(0), glm::vec3(0) });

			// [최적화] 이웃 복셀 오프셋 미리 계산
			struct Offset { int x, y, z; };
			std::vector<Offset> neighborOffsets;
			neighborOffsets.reserve((2 * influenceRadius + 1) * (2 * influenceRadius + 1) * (2 * influenceRadius + 1));

			for (int z = -influenceRadius; z <= influenceRadius; ++z)
				for (int y = -influenceRadius; y <= influenceRadius; ++y)
					for (int x = -influenceRadius; x <= influenceRadius; ++x)
						neighborOffsets.push_back({ x, y, z });


			// [최적화] OpenMP 병렬 처리
#pragma omp parallel for schedule(dynamic)
			for (int i = 0; i < (int)points.size(); ++i)
			{
				const auto& pt = points[i];

				// 포인트의 그리드 좌표 (Floating point 연산 최소화)
				glm::vec3 localPos = pt.position - aabb.min;
				int pix = (int)(localPos.x / voxelSize);
				int piy = (int)(localPos.y / voxelSize);
				int piz = (int)(localPos.z / voxelSize);

				// 미리 계산된 오프셋으로 주변 복셀 순회
				for (const auto& off : neighborOffsets)
				{
					int ix = pix + off.x;
					int iy = piy + off.y;
					int iz = piz + off.z;

					if (ix < 0 || ix >= Nx || iy < 0 || iy >= Ny || iz < 0 || iz >= Nz) continue;

					// 1D Index
					int idx = (iz * Ny + iy) * Nx + ix;

					// 복셀 중심 위치
					glm::vec3 voxelCenter = aabb.min + glm::vec3(ix + 0.5f, iy + 0.5f, iz + 0.5f) * voxelSize;

					glm::vec3 diff = voxelCenter - pt.position;

					// 거리 제곱으로 먼저 비교
					float distSq = glm::dot(diff, diff);
					if (distSq > truncSq) continue;

					float distRaw = std::sqrt(distSq);

					// TSDF 계산
					float sign = glm::dot(diff, pt.normal) < 0.0f ? -1.0f : 1.0f;
					float sdf = sign * distRaw;
					float weight = 1.0f;

					// Atomic Update
					auto& v = voxelGrid[idx];

					AtomicAddFloat(v.distSum, sdf * weight);
					AtomicAddFloat(v.weightSum, weight);

					AtomicAddFloat(v.normalSum.x, pt.normal.x * weight);
					AtomicAddFloat(v.normalSum.y, pt.normal.y * weight);
					AtomicAddFloat(v.normalSum.z, pt.normal.z * weight);

					AtomicAddFloat(v.colorSum.x, pt.color.x * weight);
					AtomicAddFloat(v.colorSum.y, pt.color.y * weight);
					AtomicAddFloat(v.colorSum.z, pt.color.z * weight);
				}
			}

			printf("Grid Integration Done. Size = %d x %d x %d\n", Nx, Ny, Nz);

			// 평균 계산 및 정규화
#pragma omp parallel for
			for (int i = 0; i < (int)voxelGrid.size(); ++i)
			{
				auto& v = voxelGrid[i];
				if (v.weightSum > 0.0001f)
				{
					v.distSum /= v.weightSum;
					v.normalSum = glm::normalize(v.normalSum);
					v.colorSum /= v.weightSum;
					v.distSum = std::clamp(v.distSum, -trunc, trunc);
				}
				else
				{
					v.distSum = 1.0f; // 외부로 설정
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

			struct ThreadBuffer {
				std::vector<MCVertex> vertices;
				std::vector<uint32_t> indices;
			};
			int maxThreads = omp_get_max_threads();
			std::vector<ThreadBuffer> threadBuffers(maxThreads);

#pragma omp parallel for collapse(3) schedule(static)
			for (int z = 0; z < Nz - 1; ++z)
			{
				for (int y = 0; y < Ny - 1; ++y)
				{
					for (int x = 0; x < Nx - 1; ++x)
					{
						int tid = omp_get_thread_num();

						// Grid Indices
						int idx[8];
						idx[0] = (z * Ny + y) * Nx + x;
						idx[1] = idx[0] + 1;
						idx[2] = idx[0] + Nx + 1;
						idx[3] = idx[0] + Nx;
						idx[4] = idx[0] + Nx * Ny;
						idx[5] = idx[4] + 1;
						idx[6] = idx[4] + Nx + 1;
						idx[7] = idx[4] + Nx;

						float cubeVal[8];
						glm::vec3 cubePos[8];
						glm::vec3 cubeNorm[8];
						glm::vec3 cubeCol[8];

						int cornerOffsets[8][3] = {
							{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
							{0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}
						};

						for (int j = 0; j < 8; ++j)
						{
							const auto& v = voxelGrid[idx[j]];
							cubeVal[j] = v.distSum;
							cubeNorm[j] = v.normalSum;
							cubeCol[j] = v.colorSum;

							cubePos[j] = aabb.min + glm::vec3(x + cornerOffsets[j][0], y + cornerOffsets[j][1], z + cornerOffsets[j][2]) * voxelSize;
						}

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

							// 1. 삼각형 노멀 계산
							glm::vec3 triNormal = glm::cross(vP[b] - vP[a], vP[c] - vP[a]);

							// 2. 저장된 평균 노멀
							glm::vec3 storedNormal(0.0f);
							int validNormals = 0;
							for (int j = 0; j < 8; ++j) {
								if (glm::length(cubeNorm[j]) > 0.01f) {
									storedNormal += cubeNorm[j];
									validNormals++;
								}
							}

							// 3. Back-Face Removal
							if (validNormals > 0) {
								if (glm::dot(triNormal, storedNormal) < 0.0f) {
									continue; // 뒷면 제거
								}
							}

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
			// ====================================================
			std::vector<MCVertex> weldedVertices;
			std::vector<uint32_t> weldedIndices;

			std::map<std::tuple<int, int, int>, uint32_t> vMap;
			float scale = 10000.0f;

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
				uint32_t targetVertex;
				int twinEdge = -1;
			};

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

					int currentEdgeIdx = (int)halfEdges.size();
					halfEdges.push_back({ v, -1 });

					uint64_t key = ((uint64_t)u << 32) | v;
					edgeMap[key] = currentEdgeIdx;

					uint64_t twinKey = ((uint64_t)v << 32) | u;
					auto it = edgeMap.find(twinKey);
					if (it != edgeMap.end())
					{
						int twinEdgeIdx = it->second;
						halfEdges[currentEdgeIdx].twinEdge = twinEdgeIdx;
						halfEdges[twinEdgeIdx].twinEdge = currentEdgeIdx;
					}
				}
			}

			// ====================================================
			// 3. Extract Borders
			// ====================================================
			std::vector<glm::vec3> borderLines;
			std::vector<glm::vec3> borderColors;

			for (auto const& [key, edgeIdx] : edgeMap)
			{
				const auto& he = halfEdges[edgeIdx];

				if (he.twinEdge == -1)
				{
					uint32_t u = (uint32_t)(key >> 32);
					uint32_t v = (uint32_t)(key & 0xFFFFFFFF);

					borderLines.push_back(weldedVertices[u].pos);
					borderLines.push_back(weldedVertices[v].pos);

					borderColors.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
					borderColors.push_back(glm::vec3(1.0f, 0.0f, 0.0f));
				}
			}

			printf("Found %zu border edges.\n", borderLines.size() / 2);

			TE(Total);

			// ====================================================
			// 4. Visualize Borders
			// ====================================================
			if (!borderLines.empty())
			{
				auto borderEntity = Feather.CreateEntity("BorderLines");
				auto lineRenderable = Feather.CreateComponent<Renderable>(borderEntity);

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