#include <libFeather.h>

using VD = VisualDebugging;
using namespace libRxTx;

const int VpB = 8;
const int VpBHalf = 4;

struct Voxel
{
	bool valid = false;
	float signedDistance = 0.0f;
	float weight = 0.0f;
	glm::vec3 normal = glm::zero<glm::vec3>();
	glm::vec3 color = glm::zero<glm::vec3>();
};

struct DataBlock
{
	glm::vec3 blockMin = glm::zero<glm::vec3>();
	Voxel voxels[VpB * VpB * VpB];

	void Initialize()
	{
		memset(voxels, 0, sizeof(Voxel) * VpB * VpB * VpB);
	}
};

typedef uint64_t DataBlockKey;

struct SparseDataBlock
{
	float voxelSize = 0.2f; // 복셀 크기가 작을수록 디테일이 살아납니다.
	glm::vec3 gridOrigin = glm::zero<glm::vec3>();
	std::unordered_map<DataBlockKey, DataBlock> dataBlocks;

	Voxel* GetVoxelByIndex(int gx, int gy, int gz)
	{
		int bx = (int)floor((float)gx / VpB);
		int by = (int)floor((float)gy / VpB);
		int bz = (int)floor((float)gz / VpB);

		float blockSize = voxelSize * VpB;
		// 정수 기반으로 정확한 블록 위치 재구성
		glm::vec3 blockMin = gridOrigin + glm::vec3((float)bx * blockSize, (float)by * blockSize, (float)bz * blockSize);

		// Key 생성
		auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSize);

		auto it = dataBlocks.find(key);
		if (it == dataBlocks.end()) return nullptr;

		int lx = gx % VpB; if (lx < 0) lx += VpB;
		int ly = gy % VpB; if (ly < 0) ly += VpB;
		int lz = gz % VpB; if (lz < 0) lz += VpB;

		return &it->second.voxels[lz * VpB * VpB + ly * VpB + lx];
	}

	void FromPLY(const std::string& filename)
	{
		TS(PLYLoading);
		PLYFormat ply;
		ply.Deserialize(filename);
		ply.FilterWithinAABB(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f);
		TE(PLYLoading);

		if (ply.GetPoints().empty()) return;

		auto [minx, miny, minz] = ply.GetAABBMin();
		// 여유 공간을 두고 Origin 설정
		glm::vec3 aabbMin(minx - 1.0f, miny - 1.0f, minz - 1.0f);

		FromPoints(
			(glm::vec3*)ply.GetPoints().data(),
			(glm::vec3*)ply.GetNormals().data(),
			(glm::vec3*)ply.GetColors().data(),
			ply.GetPoints().size() / 3,
			aabbMin);
	}

	void FromPoints(glm::vec3* points, glm::vec3* normals, glm::vec3* colors, unsigned int numberOfPoints, const glm::vec3& aabbMin)
	{
		float blockSize = voxelSize * VpB;
		gridOrigin.x = std::floor(aabbMin.x / blockSize) * blockSize;
		gridOrigin.y = std::floor(aabbMin.y / blockSize) * blockSize;
		gridOrigin.z = std::floor(aabbMin.z / blockSize) * blockSize;

		TS(Occupy);
		// 영향력 반경 (Truncation Distance): 복셀 크기보다 약간 크게
		float truncDist = voxelSize * 1.5f;

		for (size_t i = 0; i < numberOfPoints; i++)
		{
			auto p = points[i];
			glm::vec3 n = (normals) ? normals[i] : glm::vec3(0, 1, 0);
			glm::vec3 c = (colors) ? colors[i] : glm::vec3(1, 1, 1);
			if (c.r > 1.0f) c /= 255.0f;

			glm::vec3 vecFromOrigin = p - gridOrigin;
			int centerGx = (int)std::floor(vecFromOrigin.x / voxelSize);
			int centerGy = (int)std::floor(vecFromOrigin.y / voxelSize);
			int centerGz = (int)std::floor(vecFromOrigin.z / voxelSize);

			// 2x2x2 범위 (너무 넓게 퍼뜨리면 디테일이 뭉개짐, 좁으면 구멍남)
			// 정밀도를 위해 2칸씩만 검사
			for (int dz = -1; dz <= 1; ++dz) {
				for (int dy = -1; dy <= 1; ++dy) {
					for (int dx = -1; dx <= 1; ++dx) {

						int gx = centerGx + dx;
						int gy = centerGy + dy;
						int gz = centerGz + dz;
						if (gx < 0 || gy < 0 || gz < 0) continue;

						// 해당 복셀의 중심
						glm::vec3 voxelCenter = gridOrigin + glm::vec3(
							(gx + 0.5f) * voxelSize,
							(gy + 0.5f) * voxelSize,
							(gz + 0.5f) * voxelSize
						);

						// 거리 계산
						float dist = glm::distance(p, voxelCenter);

						// [핵심] 너무 먼 복셀은 건드리지 않음 (디테일 유지)
						if (dist > truncDist) continue;

						// [핵심] 가중치 계산: 가까울수록 가중치가 큼 (Linear Falloff)
						float weight = 1.0f - (dist / truncDist);

						// SDF 값 (Project point to normal)
						float sdf = glm::dot(voxelCenter - p, n);
						// SDF 값도 Truncation 범위 내로 클램핑
						if (sdf < -truncDist) sdf = -truncDist;
						if (sdf > truncDist) sdf = truncDist;

						// 블록 접근/생성
						int bx = (int)std::floor((float)gx / VpB);
						int by = (int)std::floor((float)gy / VpB);
						int bz = (int)std::floor((float)gz / VpB);
						glm::vec3 blockMin = gridOrigin + glm::vec3(bx * blockSize, by * blockSize, bz * blockSize);

						// Key 생성
						auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSize);

						if (dataBlocks.find(key) == dataBlocks.end()) {
							dataBlocks[key].Initialize();
							dataBlocks[key].blockMin = blockMin;
						}

						int lx = gx % VpB;
						int ly = gy % VpB;
						int lz = gz % VpB;
						Voxel& voxel = dataBlocks[key].voxels[lz * VpB * VpB + ly * VpB + lx];

						// 가중 평균 누적 (Weighted Running Average)
						if (voxel.weight <= 0.0001f) {
							voxel.signedDistance = sdf;
							voxel.color = c;
							voxel.weight = weight;
							voxel.valid = true;
						}
						else {
							float newW = voxel.weight + weight;
							voxel.signedDistance = (voxel.signedDistance * voxel.weight + sdf * weight) / newW;
							voxel.color = (voxel.color * voxel.weight + c * weight) / newW;
							voxel.weight = newW;
						}
					}
				}
			}
		}
		TE(Occupy);
	}
};

struct Point3D {
	float x, y, z;
	bool operator<(const Point3D& other) const { return std::tie(x, y, z) < std::tie(other.x, other.y, other.z); }
};

struct Triangle {
	glm::vec3 v[3];
	glm::vec3 c[3];
};

struct MeshGenerator
{
	std::vector<Triangle> triangles;
	std::vector<std::pair<glm::vec3, glm::vec3>> holeEdges;

	const glm::vec3 cornerOffsets[8] = {
		{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1},
		{0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}
	};

	glm::vec3 VertexInterp(float isolevel, glm::vec3 p1, glm::vec3 p2, float val1, float val2) {
		if (std::abs(val1 - val2) < 0.00001f) return p1;
		float mu = (isolevel - val1) / (val2 - val1);
		return p1 + mu * (p2 - p1);
	}
	glm::vec3 ColorInterp(float isolevel, glm::vec3 c1, glm::vec3 c2, float val1, float val2) {
		if (std::abs(val1 - val2) < 0.00001f) return c1;
		float mu = (isolevel - val1) / (val2 - val1);
		return c1 + mu * (c2 - c1);
	}

	void GenerateMeshFromBlocks(SparseDataBlock& sdb)
	{
		triangles.clear();
		holeEdges.clear();
		float isoLevel = 0.0f;

		for (auto& [key, block] : sdb.dataBlocks)
		{
			// 현재 블록의 전역 정수 인덱스 시작점 계산
			glm::vec3 diff = block.blockMin - sdb.gridOrigin;
			int startGx = (int)std::floor(diff.x / sdb.voxelSize + 0.5f); // 반올림으로 오차 제거
			int startGy = (int)std::floor(diff.y / sdb.voxelSize + 0.5f);
			int startGz = (int)std::floor(diff.z / sdb.voxelSize + 0.5f);

			for (int z = 0; z < VpB; ++z) {
				for (int y = 0; y < VpB; ++y) {
					for (int x = 0; x < VpB; ++x) {

						float vals[8];
						glm::vec3 pos[8];
						glm::vec3 cols[8];

						float sumDist = 0.0f;
						glm::vec3 sumColor(0.0f);
						int validCnt = 0; // [선언] validCnt

						// 1. 정수 인덱스로 데이터 수집
						const Voxel* voxels[8] = { nullptr };
						for (int i = 0; i < 8; ++i) {
							int gx = startGx + x + (int)cornerOffsets[i].x;
							int gy = startGy + y + (int)cornerOffsets[i].y;
							int gz = startGz + z + (int)cornerOffsets[i].z;

							pos[i] = sdb.gridOrigin + glm::vec3(gx, gy, gz) * sdb.voxelSize;

							// 정수 인덱스 조회 (경계면 일치 보장)
							voxels[i] = sdb.GetVoxelByIndex(gx, gy, gz);

							if (voxels[i] && voxels[i]->valid && voxels[i]->weight > 0.001f) {
								sumDist += voxels[i]->signedDistance;
								sumColor += voxels[i]->color;
								validCnt++; // [사용] validCnt
							}
						}

						// [수정] validCount -> validCnt 로 통일
						if (validCnt == 0) continue;

						// 빈 공간 채우기 (Gap Closing)
						float avgDist = sumDist / validCnt;
						glm::vec3 avgColor = sumColor / (float)validCnt;

						for (int i = 0; i < 8; ++i) {
							if (voxels[i] && voxels[i]->valid && voxels[i]->weight > 0.001f) {
								vals[i] = voxels[i]->signedDistance;
								cols[i] = voxels[i]->color;
							}
							else {
								vals[i] = avgDist;
								cols[i] = avgColor;
							}
						}

						// -- 마칭 큐브 로직 --
						int idx = 0;
						if (vals[0] < isoLevel) idx |= 1;
						if (vals[1] < isoLevel) idx |= 2;
						if (vals[2] < isoLevel) idx |= 4;
						if (vals[3] < isoLevel) idx |= 8;
						if (vals[4] < isoLevel) idx |= 16;
						if (vals[5] < isoLevel) idx |= 32;
						if (vals[6] < isoLevel) idx |= 64;
						if (vals[7] < isoLevel) idx |= 128;

						if (edgeTable[idx] == 0) continue;

						glm::vec3 vList[12], cList[12];
						auto Interp = [&](int ei, int v1, int v2) {
							if (edgeTable[idx] & (1 << ei)) {
								vList[ei] = VertexInterp(isoLevel, pos[v1], pos[v2], vals[v1], vals[v2]);
								cList[ei] = ColorInterp(isoLevel, cols[v1], cols[v2], vals[v1], vals[v2]);
							}
							};

						Interp(0, 0, 1); Interp(1, 1, 2); Interp(2, 2, 3); Interp(3, 3, 0);
						Interp(4, 4, 5); Interp(5, 5, 6); Interp(6, 6, 7); Interp(7, 7, 4);
						Interp(8, 0, 4); Interp(9, 1, 5); Interp(10, 2, 6); Interp(11, 3, 7);

						for (int i = 0; triTable[idx][i] != -1; i += 3) {
							Triangle tri;
							for (int k = 0; k < 3; ++k) {
								int id = triTable[idx][i + k];
								tri.v[k] = vList[id];
								tri.c[k] = cList[id];
							}
							triangles.push_back(tri);
						}
					}
				}
			}
		}
	}

	void SmoothMesh(int iterations = 3)
	{
		if (triangles.empty()) return;

		// Welding Tolerance를 넉넉하게 줌 (틈새가 있어도 붙도록)
		float tol = 0.001f;

		std::vector<glm::vec3> uniqueVerts;
		std::vector<glm::vec3> uniqueColors;
		std::vector<uint32_t> indices;
		std::map<std::tuple<int, int, int>, uint32_t> vMap;

		for (const auto& t : triangles) {
			for (int i = 0; i < 3; ++i) {
				std::tuple<int, int, int> key = { (int)(t.v[i].x / tol), (int)(t.v[i].y / tol), (int)(t.v[i].z / tol) };
				if (vMap.find(key) == vMap.end()) {
					vMap[key] = (uint32_t)uniqueVerts.size();
					uniqueVerts.push_back(t.v[i]);
					uniqueColors.push_back(t.c[i]);
				}
				indices.push_back(vMap[key]);
			}
		}

		// Laplacian Smoothing
		std::vector<std::vector<uint32_t>> adj(uniqueVerts.size());
		for (size_t i = 0; i < indices.size(); i += 3) {
			uint32_t i0 = indices[i], i1 = indices[i + 1], i2 = indices[i + 2];
			adj[i0].push_back(i1); adj[i0].push_back(i2);
			adj[i1].push_back(i0); adj[i1].push_back(i2);
			adj[i2].push_back(i0); adj[i2].push_back(i1);
		}

		for (int iter = 0; iter < iterations; ++iter) {
			std::vector<glm::vec3> smoothed = uniqueVerts;
			for (size_t i = 0; i < uniqueVerts.size(); ++i) {
				if (adj[i].empty()) continue;
				glm::vec3 avg(0.0f);
				for (uint32_t n : adj[i]) avg += uniqueVerts[n];
				avg /= (float)adj[i].size();
				smoothed[i] = glm::mix(uniqueVerts[i], avg, 0.5f);
			}
			uniqueVerts = smoothed;
		}

		triangles.clear();
		for (size_t i = 0; i < indices.size(); i += 3) {
			Triangle tri;
			tri.v[0] = uniqueVerts[indices[i]];
			tri.v[1] = uniqueVerts[indices[i + 1]];
			tri.v[2] = uniqueVerts[indices[i + 2]];
			tri.c[0] = uniqueColors[indices[i]];
			tri.c[1] = uniqueColors[indices[i + 1]];
			tri.c[2] = uniqueColors[indices[i + 2]];
			triangles.push_back(tri);
		}
	}

	void DetectHoles()
	{
		// 엣지 공유 횟수 카운트 (스무딩 된 데이터 기반)
		// 단순하게 좌표 기반으로 다시 카운트합니다.
		std::map<std::tuple<int, int, int>, uint32_t> vMap;
		std::vector<glm::vec3> verts;
		float tol = 0.005f; // 노이즈 무시할 웰딩 허용치

		for (const auto& t : triangles) {
			for (int i = 0; i < 3; ++i) {
				std::tuple<int, int, int> key = { (int)(t.v[i].x / tol), (int)(t.v[i].y / tol), (int)(t.v[i].z / tol) };
				if (vMap.find(key) == vMap.end()) {
					vMap[key] = (uint32_t)verts.size();
					verts.push_back(t.v[i]);
				}
			}
		}

		std::map<std::pair<uint32_t, uint32_t>, int> edgeCount;
		for (const auto& t : triangles) {
			uint32_t idx[3];
			for (int i = 0; i < 3; ++i) {
				std::tuple<int, int, int> key = { (int)(t.v[i].x / tol), (int)(t.v[i].y / tol), (int)(t.v[i].z / tol) };
				idx[i] = vMap[key];
			}
			for (int j = 0; j < 3; ++j) {
				uint32_t a = idx[j], b = idx[(j + 1) % 3];
				if (a > b) std::swap(a, b);
				edgeCount[{a, b}]++;
			}
		}

		holeEdges.clear();
		for (auto& kv : edgeCount) {
			if (kv.second == 1) {
				glm::vec3 p1 = verts[kv.first.first];
				glm::vec3 p2 = verts[kv.first.second];
				// 너무 짧은 엣지(짜투리)는 무시
				if (glm::distance(p1, p2) > 0.01f) {
					holeEdges.push_back({ p1, p2 });
				}
			}
		}
	}

	void Visualize(bool showMesh, bool showHoles)
	{
		if (showMesh) {
			for (const auto& tri : triangles) {
				glm::vec3 avgC = (tri.c[0] + tri.c[1] + tri.c[2]) / 3.0f;
				VD::AddTriangle("Mesh", tri.v[0], tri.v[1], tri.v[2], glm::vec4(avgC, 1.0f));
			}
		}
		if (showHoles) {
			for (const auto& edge : holeEdges) {
				VD::AddLine("Holes", edge.first, edge.second, Color::magenta());
			}
		}
	}
};

static inline std::string FormatWithCommas(size_t value)
{
	std::string numStr = std::to_string(value);
	int insertPosition = static_cast<int>(numStr.length()) - 3;
	while (insertPosition > 0)
	{
		numStr.insert(insertPosition, ",");
		insertPosition -= 3;
	}
	return numStr;
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
			});
	}
#pragma endregion

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

	Feather.AddOnInitializeCallback([&]() {
		{
			SparseDataBlock sdb;
			sdb.FromPLY("D:\\Debug\\PLY\\inputA.ply");

			//sdb.ShowBlocks();

			TS(MeshGeneration);

			static MeshGenerator meshGen;
			meshGen.GenerateMeshFromBlocks(sdb); // 실제 테이블 사용
			//meshGen.SmoothMesh(3);
			meshGen.DetectHoles();
			meshGen.Visualize(true, true);

			TE(MeshGeneration);

			alog("Total DataBlocks : %s\n", FormatWithCommas(sdb.dataBlocks.size()).c_str());
			alog("DataBlock Size : %zd\n", sizeof(DataBlock));
			alog("Total Memory : %s bytes\n", FormatWithCommas(sdb.dataBlocks.size() * sizeof(DataBlock)).c_str());
		}

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

		});

	Feather.Run();
	Feather.Terminate();

	return 0;
}