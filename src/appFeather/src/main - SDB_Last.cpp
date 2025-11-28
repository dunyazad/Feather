#include <libFeather.h>

using VD = VisualDebugging;
using namespace libRxTx;

const int VpB = 8;
const int VpBHalf = 4;

struct Voxel
{
	bool valid = false;
	float signedDistance = FLT_MAX;
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

	Voxel* FindVoxel(const glm::vec3& position, float voxelSize)
	{
		const float blockSize = voxelSize * VpB;
		const float epsilon = 1e-4f; // 부동 소수점 오차 허용 범위

		glm::vec3 delta = position - blockMin;

		// half-open interval: 0 <= delta < blockSize
		// Epsilon을 사용하여 경계에 걸친 점들을 포용합니다.
		if (delta.x < -epsilon || delta.x >= blockSize + epsilon ||
			delta.y < -epsilon || delta.y >= blockSize + epsilon ||
			delta.z < -epsilon || delta.z >= blockSize + epsilon)
		{
			return nullptr;
		}

		int xIndex = (int)floorf(delta.x / voxelSize);
		int yIndex = (int)floorf(delta.y / voxelSize);
		int zIndex = (int)floorf(delta.z / voxelSize);

		// 오차로 인해 인덱스가 -1이 되거나 VpB가 되는 경우를 방지하기 위해 클램핑(Clamping) 처리
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
		ply.FilterWithinAABB(
			-10.0f, -10.0f, -10.0f,
			10.0f, 10.0f, 10.0f);
		TE(PLYLoading);

		auto [minx, miny, minz] = ply.GetAABBMin();
		glm::vec3 aabbMin(minx, miny, minz);

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
			auto key = Morton3D::EncodeFromVec3(p, gridOrigin, voxelSize * VpB);

			uint32_t bx, by, bz;
			Morton3D::Decode(key, bx, by, bz);
			glm::vec3 min = gridOrigin +
				glm::vec3(
					(float)bx * voxelSize * VpB,
					(float)by * voxelSize * VpB,
					(float)bz * voxelSize * VpB
				);
			if (dataBlocks.find(key) == dataBlocks.end())
			{
				dataBlocks[key].Initialize();
				dataBlocks[key].blockMin = min;
			}
			auto voxel = dataBlocks[key].FindVoxel(p, voxelSize);
			if (nullptr != voxel)
			{
				voxel->valid = true;
			}
			else
			{
				glm::vec3 blockMin = dataBlocks[key].blockMin;
				glm::vec3 blockMax = blockMin + glm::vec3(VpB * voxelSize);

				alog("Error! p=(%.3f %.3f %.3f)  block=[%.3f %.3f %.3f]~[%.3f %.3f %.3f]\n",
					p.x, p.y, p.z,
					blockMin.x, blockMin.y, blockMin.z,
					blockMax.x, blockMax.y, blockMax.z);
			}

			//auto n = glm::vec3(ply.GetNormals()[i * 3 + 0], ply.GetNormals()[i * 3 + 1], ply.GetNormals()[i * 3 + 2]);
			//if (false == ply.GetColors().empty())
			//{
			//	auto c = glm::vec3(ply.GetColors()[i * 3 + 0], ply.GetColors()[i * 3 + 1], ply.GetColors()[i * 3 + 2]);
			//
			//	//VD::AddSphere("point", p, 0.01f, glm::vec4(c, 1.0f));
			//}
			//else
			//{
			//	//VD::AddSphere("point", p, 0.01f, Color::white());
			//}
		}
		TE(Occupy);
	}

	void ShowBlocks()
	{
		for (auto& [key, block] : dataBlocks)
		{
			auto center = block.blockMin + glm::vec3(voxelSize * VpBHalf, voxelSize * VpBHalf, voxelSize * VpBHalf);
			VD::AddWiredBox("datablock", center, glm::vec3(voxelSize * VpB, voxelSize * VpB, voxelSize * VpB), Color::black());

			for (size_t z = 0; z < VpB; z++)
			{
				for (size_t y = 0; y < VpB; y++)
				{
					for (size_t x = 0; x < VpB; x++)
					{
						auto voxel = block.voxels[z * VpB * VpB + y * VpB + x];
						if (voxel.valid)
						{
							auto vcenter = block.blockMin
								+ glm::vec3((float)x * voxelSize, (float)y * voxelSize, (float)z * voxelSize)
								+ glm::vec3(0.5f * voxelSize, 0.5f * voxelSize, 0.5f * voxelSize);

							VD::AddWiredBox("datablock", vcenter, glm::vec3(voxelSize, voxelSize, voxelSize), Color::yellow());
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