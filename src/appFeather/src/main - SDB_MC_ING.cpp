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
	float voxelSize = 0.2f;
	glm::vec3 gridOrigin = glm::zero<glm::vec3>();
	std::unordered_map<DataBlockKey, DataBlock> dataBlocks;

	void FromPLY(const std::string& filename)
	{
		TS(PLYLoading);
		PLYFormat ply;
		ply.Deserialize(filename);
		ply.FilterWithinAABB(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f);
		TE(PLYLoading);

		auto [minx, miny, minz] = ply.GetAABBMin();
		glm::vec3 aabbMin(minx, miny, minz);

		if (ply.GetPoints().empty()) return;

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
		// AABB Min을 기준으로 넉넉하게 그리드 시작점을 잡음
		gridOrigin.x = std::floor(aabbMin.x / blockSize) * blockSize;
		gridOrigin.y = std::floor(aabbMin.y / blockSize) * blockSize;
		gridOrigin.z = std::floor(aabbMin.z / blockSize) * blockSize;

		TS(Occupy);
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

			// Splatting: 점 하나를 주변 3x3x3 복셀로 퍼뜨림 (부드러운 연결)
			for (int dz = -1; dz <= 1; ++dz) {
				for (int dy = -1; dy <= 1; ++dy) {
					for (int dx = -1; dx <= 1; ++dx) {

						int gx = centerGx + dx;
						int gy = centerGy + dy;
						int gz = centerGz + dz;

						// [핵심 수정] 폭발 방지: 음수 인덱스가 나오면 Morton Code가 터지므로 무시
						if (gx < 0 || gy < 0 || gz < 0) continue;

						int bx = (int)std::floor((float)gx / VpB);
						int by = (int)std::floor((float)gy / VpB);
						int bz = (int)std::floor((float)gz / VpB);

						int lx = gx % VpB;
						int ly = gy % VpB;
						int lz = gz % VpB;

						glm::vec3 blockMin = gridOrigin + glm::vec3(bx * blockSize, by * blockSize, bz * blockSize);
						// 안전한 키 생성을 위해 블록 중심점 근처 사용
						auto key = Morton3D::EncodeFromVec3(blockMin + glm::vec3(voxelSize * 0.1f), gridOrigin, blockSize);

						if (dataBlocks.find(key) == dataBlocks.end()) {
							dataBlocks[key].Initialize();
							dataBlocks[key].blockMin = blockMin;
						}

						Voxel& voxel = dataBlocks[key].voxels[lz * VpB * VpB + ly * VpB + lx];

						// 해당 복셀의 중심
						glm::vec3 voxelCenter = blockMin + glm::vec3((lx + 0.5f) * voxelSize, (ly + 0.5f) * voxelSize, (lz + 0.5f) * voxelSize);

						float dist = glm::dot(voxelCenter - p, n);

						// 너무 먼 곳은 영향 주지 않음 (노이즈 방지)
						if (std::abs(dist) > voxelSize * 1.5f) continue;

						// 가중치 평균 (Weighted Average)
						if (voxel.weight <= 0.0f) {
							voxel.signedDistance = dist;
							voxel.color = c;
							voxel.normal = n;
							voxel.weight = 1.0f;
							voxel.valid = true;
						}
						else {
							float newW = voxel.weight + 1.0f;
							voxel.signedDistance = (voxel.signedDistance * voxel.weight + dist) / newW;
							voxel.color = (voxel.color * voxel.weight + c) / newW;
							voxel.normal = glm::normalize(voxel.normal * voxel.weight + n);
							voxel.weight = newW;
						}
					}
				}
			}
		}
		TE(Occupy);
	}
};

// ... (Triangle, Point3D 등 헬퍼 구조체 유지) ...
struct Point3D {
	float x, y, z;
	bool operator<(const Point3D& other) const { return std::tie(x, y, z) < std::tie(other.x, other.y, other.z); }
};
struct Triangle {
	glm::vec3 v[3];
	glm::vec3 c[3]; // 색상 포함
};

struct MeshGenerator
{
	std::vector<Triangle> triangles;
	std::vector<std::pair<glm::vec3, glm::vec3>> holeEdges;

	const glm::vec3 cornerOffsets[8] = {
		{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1},
		{0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}
	};

	// 보간 함수들
	glm::vec3 VertexInterp(float isolevel, glm::vec3 p1, glm::vec3 p2, float val1, float val2)
	{
		if (std::abs(val1 - val2) < 0.00001f) return p1;
		float mu = (isolevel - val1) / (val2 - val1);
		return p1 + mu * (p2 - p1);
	}

	glm::vec3 ColorInterp(float isolevel, glm::vec3 c1, glm::vec3 c2, float val1, float val2)
	{
		if (std::abs(val1 - val2) < 0.00001f) return c1;
		float mu = (isolevel - val1) / (val2 - val1);
		return c1 + mu * (c2 - c1);
	}

	void GenerateMeshFromBlocks(SparseDataBlock& sdb)
	{
		triangles.clear();
		holeEdges.clear();
		float isoLevel = 0.0f;

		// 람다: 월드 좌표로 복셀 조회
		auto GetVoxelAtWorldPos = [&](glm::vec3 pos) -> const Voxel* {
			glm::vec3 offset = pos - sdb.gridOrigin;
			if (offset.x < 0 || offset.y < 0 || offset.z < 0) return nullptr;

			auto key = Morton3D::EncodeFromVec3(pos, sdb.gridOrigin, sdb.voxelSize * VpB);
			auto it = sdb.dataBlocks.find(key);
			if (it == sdb.dataBlocks.end()) return nullptr;

			glm::vec3 localDelta = pos - it->second.blockMin;
			int lx = (int)std::floor(localDelta.x / sdb.voxelSize);
			int ly = (int)std::floor(localDelta.y / sdb.voxelSize);
			int lz = (int)std::floor(localDelta.z / sdb.voxelSize);

			if (lx >= 0 && lx < VpB && ly >= 0 && ly < VpB && lz >= 0 && lz < VpB) {
				return &it->second.voxels[lz * VpB * VpB + ly * VpB + lx];
			}
			return nullptr;
			};

		for (auto& [key, block] : sdb.dataBlocks)
		{
			for (int z = 0; z < VpB; ++z) {
				for (int y = 0; y < VpB; ++y) {
					for (int x = 0; x < VpB; ++x) {

						float cubeValues[8];
						glm::vec3 cubePos[8];
						glm::vec3 cubeColors[8];

						// [핵심] 8개 코너가 모두 유효한 데이터인지 확인
						bool fullyValid = true;

						for (int i = 0; i < 8; ++i) {
							int lx = x + (int)cornerOffsets[i].x;
							int ly = y + (int)cornerOffsets[i].y;
							int lz = z + (int)cornerOffsets[i].z;
							cubePos[i] = block.blockMin + glm::vec3(lx * sdb.voxelSize, ly * sdb.voxelSize, lz * sdb.voxelSize);

							const Voxel* v = GetVoxelAtWorldPos(cubePos[i] + glm::vec3(sdb.voxelSize * 0.5f));

							if (v && v->valid) {
								cubeValues[i] = v->signedDistance;
								cubeColors[i] = v->color;
							}
							else {
								// 하나라도 데이터가 끊기면(=경계면이면) 아예 그리지 않음
								// 이것이 '뒷면 벽'을 제거하는 가장 확실한 방법입니다.
								fullyValid = false;
								break;
							}
						}

						if (!fullyValid) continue;

						// -- 이하 마칭 큐브 로직 동일 --
						int cubeIndex = 0;
						if (cubeValues[0] < isoLevel) cubeIndex |= 1;
						if (cubeValues[1] < isoLevel) cubeIndex |= 2;
						if (cubeValues[2] < isoLevel) cubeIndex |= 4;
						if (cubeValues[3] < isoLevel) cubeIndex |= 8;
						if (cubeValues[4] < isoLevel) cubeIndex |= 16;
						if (cubeValues[5] < isoLevel) cubeIndex |= 32;
						if (cubeValues[6] < isoLevel) cubeIndex |= 64;
						if (cubeValues[7] < isoLevel) cubeIndex |= 128;

						if (edgeTable[cubeIndex] == 0) continue;

						glm::vec3 vertList[12];
						glm::vec3 colorList[12];

						auto Interp = [&](int edgeIdx, int v1, int v2) {
							if (edgeTable[cubeIndex] & (1 << edgeIdx)) {
								vertList[edgeIdx] = VertexInterp(isoLevel, cubePos[v1], cubePos[v2], cubeValues[v1], cubeValues[v2]);
								colorList[edgeIdx] = ColorInterp(isoLevel, cubeColors[v1], cubeColors[v2], cubeValues[v1], cubeValues[v2]);
							}
							};

						Interp(0, 0, 1); Interp(1, 1, 2); Interp(2, 2, 3); Interp(3, 3, 0);
						Interp(4, 4, 5); Interp(5, 5, 6); Interp(6, 6, 7); Interp(7, 7, 4);
						Interp(8, 0, 4); Interp(9, 1, 5); Interp(10, 2, 6); Interp(11, 3, 7);

						for (int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
							Triangle tri;
							for (int k = 0; k < 3; ++k) {
								int idx = triTable[cubeIndex][i + k];
								tri.v[k] = vertList[idx];
								tri.c[k] = colorList[idx];
							}
							triangles.push_back(tri);
						}
					}
				}
			}
		}
	}

	// [Welding 적용된 DetectHoles 유지]
	void DetectHoles()
	{
		std::vector<glm::vec3> weldedVertices;
		std::vector<uint32_t> indices;
		std::map<std::tuple<int, int, int>, uint32_t> uniqueMap;
		float tolerance = 0.005f;

		for (const auto& tri : triangles) {
			for (int i = 0; i < 3; ++i) {
				std::tuple<int, int, int> key = { (int)(tri.v[i].x / tolerance), (int)(tri.v[i].y / tolerance), (int)(tri.v[i].z / tolerance) };
				if (uniqueMap.find(key) == uniqueMap.end()) {
					uniqueMap[key] = (uint32_t)weldedVertices.size();
					weldedVertices.push_back(tri.v[i]);
				}
				indices.push_back(uniqueMap[key]);
			}
		}
		std::map<std::pair<uint32_t, uint32_t>, int> edgeCount;
		for (size_t i = 0; i < indices.size(); i += 3) {
			uint32_t idx[3] = { indices[i], indices[i + 1], indices[i + 2] };
			for (int j = 0; j < 3; ++j) {
				uint32_t a = idx[j], b = idx[(j + 1) % 3];
				if (a > b) std::swap(a, b);
				edgeCount[{a, b}]++;
			}
		}
		holeEdges.clear();
		for (auto& kv : edgeCount) {
			if (kv.second == 1) holeEdges.push_back({ weldedVertices[kv.first.first], weldedVertices[kv.first.second] });
		}
	}

	void Visualize(bool showMesh, bool showHoles)
	{
		if (showMesh) {
			for (const auto& tri : triangles) {
				glm::vec3 avgColor = (tri.c[0] + tri.c[1] + tri.c[2]) / 3.0f;
				// 흰색 메쉬 + 원래 색상 적용
				VD::AddTriangle("Mesh", tri.v[0], tri.v[1], tri.v[2], glm::vec4(avgColor, 1.0f));
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