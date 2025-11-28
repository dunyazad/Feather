#include <libFeather.h>

using VD = VisualDebugging;
using namespace libRxTx;

const int VpB = 8;
const int VpBHalf = 4;

struct Voxel
{
	bool valid = false;
	float signedDistance = 0.0f; // 초기화 값을 0.0f로 변경 (계산 로직 단순화)
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

	// 인덱스 계산 로직을 명확히 하기 위해 내부 로직은 유지하되,
	// 외부에서 Voxel 포인터를 얻는 기능은 그대로 둡니다.
	Voxel* FindVoxel(const glm::vec3& position, float voxelSize)
	{
		const float blockSize = voxelSize * VpB;
		const float epsilon = 1e-4f;

		glm::vec3 delta = position - blockMin;

		if (delta.x < -epsilon || delta.x >= blockSize + epsilon ||
			delta.y < -epsilon || delta.y >= blockSize + epsilon ||
			delta.z < -epsilon || delta.z >= blockSize + epsilon)
		{
			return nullptr;
		}

		int xIndex = (int)floorf(delta.x / voxelSize);
		int yIndex = (int)floorf(delta.y / voxelSize);
		int zIndex = (int)floorf(delta.z / voxelSize);

		if (xIndex < 0) xIndex = 0;
		if (xIndex >= VpB) xIndex = VpB - 1;

		if (yIndex < 0) yIndex = 0;
		if (yIndex >= VpB) yIndex = VpB - 1;

		if (zIndex < 0) zIndex = 0;
		if (zIndex >= VpB) zIndex = VpB - 1;

		return &voxels[zIndex * VpB * VpB + yIndex * VpB + xIndex];
	}
};

typedef uint64_t DataBlockKey;

struct SparseDataBlock
{
	float voxelSize = 0.2f;
	std::unordered_map<DataBlockKey, DataBlock> dataBlocks;

	void FromPLY(const std::string& filename)
	{
		TS(PLYLoading);
		PLYFormat ply;
		ply.Deserialize(filename);
		// 필요한 경우 필터링 수행
		ply.FilterWithinAABB(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f);
		TE(PLYLoading);

		auto [minx, miny, minz] = ply.GetAABBMin();
		glm::vec3 aabbMin(minx, miny, minz);

		// PLY 데이터가 비어있지 않은지 확인
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
		glm::vec3 gridOrigin;
		float blockSize = voxelSize * VpB;

		gridOrigin.x = std::floor(aabbMin.x / blockSize) * blockSize;
		gridOrigin.y = std::floor(aabbMin.y / blockSize) * blockSize;
		gridOrigin.z = std::floor(aabbMin.z / blockSize) * blockSize;

		TS(Occupy);
		for (size_t i = 0; i < numberOfPoints; i++)
		{
			auto p = points[i];

			// 노멀 데이터가 없는 경우를 대비한 안전장치 (기본값 Y축)
			glm::vec3 n = (normals) ? normals[i] : glm::vec3(0, 1, 0);
			// 컬러 데이터가 없는 경우 기본값 (White)
			glm::vec3 c = (colors) ? colors[i] : glm::vec3(1, 1, 1);

			auto key = Morton3D::EncodeFromVec3(p, gridOrigin, voxelSize * VpB);

			// 블록이 없으면 생성
			if (dataBlocks.find(key) == dataBlocks.end())
			{
				uint32_t bx, by, bz;
				Morton3D::Decode(key, bx, by, bz);
				glm::vec3 min = gridOrigin +
					glm::vec3(
						(float)bx * voxelSize * VpB,
						(float)by * voxelSize * VpB,
						(float)bz * voxelSize * VpB
					);

				dataBlocks[key].Initialize();
				dataBlocks[key].blockMin = min;
			}

			DataBlock& currentBlock = dataBlocks[key];

			// 복셀 인덱스 직접 계산 (FindVoxel 로직 인라인화)
			// 중심점(Center) 계산을 위해 인덱스가 필요함
			glm::vec3 delta = p - currentBlock.blockMin;
			int xIndex = (int)floorf(delta.x / voxelSize);
			int yIndex = (int)floorf(delta.y / voxelSize);
			int zIndex = (int)floorf(delta.z / voxelSize);

			// 클램핑
			if (xIndex < 0) xIndex = 0; if (xIndex >= VpB) xIndex = VpB - 1;
			if (yIndex < 0) yIndex = 0; if (yIndex >= VpB) yIndex = VpB - 1;
			if (zIndex < 0) zIndex = 0; if (zIndex >= VpB) zIndex = VpB - 1;

			// 해당 복셀 가져오기
			Voxel& voxel = currentBlock.voxels[zIndex * VpB * VpB + yIndex * VpB + xIndex];

			// 1. 복셀의 중심(Center) 위치 계산
			glm::vec3 voxelCenter = currentBlock.blockMin + glm::vec3(
				(xIndex + 0.5f) * voxelSize,
				(yIndex + 0.5f) * voxelSize,
				(zIndex + 0.5f) * voxelSize
			);

			// 2. SDF 계산 (Point Project to Normal)
			// 평면 근사: (VoxelCenter - Point) dot Normal
			float dist = glm::dot(voxelCenter - p, n);

			// 3. TSDF 통합 (Running Average)
			const float currentWeight = 1.0f; // 현재 샘플의 가중치

			if (voxel.weight <= 0.0f)
			{
				// 첫 방문: 그대로 대입
				voxel.signedDistance = dist;
				voxel.color = c;
				voxel.normal = n;
				voxel.weight = currentWeight;
				voxel.valid = true;
			}
			else
			{
				// 누적 평균 (Weighted Average)
				// NewAverage = (OldAverage * OldWeight + NewValue * NewWeight) / (OldWeight + NewWeight)
				float newTotalWeight = voxel.weight + currentWeight;

				voxel.signedDistance = (voxel.signedDistance * voxel.weight + dist * currentWeight) / newTotalWeight;
				voxel.color = (voxel.color * voxel.weight + c * currentWeight) / newTotalWeight;

				// Normal은 단순히 더해서 정규화 (방향 평균)
				voxel.normal = glm::normalize(voxel.normal * voxel.weight + n * currentWeight);

				voxel.weight = newTotalWeight;
			}
		}
		TE(Occupy);
	}

	void ShowBlocks()
	{
		for (auto& [key, block] : dataBlocks)
		{
			// 블록 바운딩 박스 (검은색)
			auto center = block.blockMin + glm::vec3(voxelSize * VpBHalf, voxelSize * VpBHalf, voxelSize * VpBHalf);
			VD::AddWiredBox("datablock", center, glm::vec3(voxelSize * VpB, voxelSize * VpB, voxelSize * VpB), Color::black());

			for (size_t z = 0; z < VpB; z++)
			{
				for (size_t y = 0; y < VpB; y++)
				{
					for (size_t x = 0; x < VpB; x++)
					{
						auto& voxel = block.voxels[z * VpB * VpB + y * VpB + x];
						if (voxel.valid)
						{
							auto vcenter = block.blockMin
								+ glm::vec3((float)x * voxelSize, (float)y * voxelSize, (float)z * voxelSize)
								+ glm::vec3(0.5f * voxelSize, 0.5f * voxelSize, 0.5f * voxelSize);

							// 시각화: SDF 값에 따라 색상 변화를 주거나 크기를 조절할 수 있습니다.
							// 여기서는 Color 값을 그대로 사용합니다.
							glm::vec4 displayColor = glm::vec4(voxel.color, 1.0f);

							// SDF 값이 0에 가까울수록(표면) 박스를 작게 그리는 예시 (선택사항)
							// float scale = 1.0f - glm::clamp(std::abs(voxel.signedDistance) / voxelSize, 0.0f, 0.8f);
							VD::AddWiredBox("datablock", vcenter, glm::vec3(voxelSize, voxelSize, voxelSize), displayColor);
						}
					}
				}
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

			sdb.ShowBlocks();

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