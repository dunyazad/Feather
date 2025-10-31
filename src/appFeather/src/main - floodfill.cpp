#include <iostream>

#include <libFeather.h>

using VD = VisualDebugging;


f32 voxelSize = 0.1f;

struct VoxelKey
{
	i32 x = 0;
	i32 y = 0;
	i32 z = 0;

	bool operator==(const VoxelKey& other) const
	{
		return x == other.x && y == other.y && z == other.z;
	}

	static inline uint64_t expandBits(uint32_t v) noexcept
	{
		v = (v | (v << 16)) & 0x030000FF;
		v = (v | (v << 8)) & 0x0300F00F;
		v = (v | (v << 4)) & 0x030C30C3;
		v = (v | (v << 2)) & 0x09249249;
		return v;
	}

	static inline uint32_t compactBits(uint64_t v) noexcept
	{
		v &= 0x09249249;
		v = (v ^ (v >> 2)) & 0x030C30C3;
		v = (v ^ (v >> 4)) & 0x0300F00F;
		v = (v ^ (v >> 8)) & 0x030000FF;
		v = (v ^ (v >> 16)) & 0x000003FF;
		return static_cast<uint32_t>(v);
	}

	uint64_t encode() const noexcept
	{
		// 1048576는 2^20 으로, 음수를 양수로 변환하기 위한 offset
		uint32_t ux = static_cast<uint32_t>(x + 1048576) & 0x1FFFFF; // 21-bit
		uint32_t uy = static_cast<uint32_t>(y + 1048576) & 0x1FFFFF;
		uint32_t uz = static_cast<uint32_t>(z + 1048576) & 0x1FFFFF;

		uint64_t xx = expandBits(ux);
		uint64_t yy = expandBits(uy);
		uint64_t zz = expandBits(uz);
		return xx | (yy << 1) | (zz << 2);
	}

	void decode(uint64_t code) noexcept
	{
		x = static_cast<i32>(compactBits(code)) - 1048576;
		y = static_cast<i32>(compactBits(code >> 1)) - 1048576;
		z = static_cast<i32>(compactBits(code >> 2)) - 1048576;
	}

	static VoxelKey ToVoxelKey(const glm::vec3& pos, f32 voxelSize)
	{
		return {
			static_cast<i32>(floorf(pos.x / voxelSize)),
			static_cast<i32>(floorf(pos.y / voxelSize)),
			static_cast<i32>(floorf(pos.z / voxelSize))
		};
	}

	static glm::vec3 FromVoxelKey(const VoxelKey& key, f32 voxelSize)
	{
		return glm::vec3(
			(key.x + 0.5f) * voxelSize,
			(key.y + 0.5f) * voxelSize,
			(key.z + 0.5f) * voxelSize
		);
	}
};

struct Voxel
{
	ui64 hash = 0;
	std::vector<ui32> pointIndices;
	ui32 label = invalid_ui32;
};

namespace std
{
	template<>
	struct hash<VoxelKey>
	{
		size_t operator()(const VoxelKey& k) const noexcept
		{
			return static_cast<size_t>(k.encode());
		}
	};
}

std::unordered_map<VoxelKey, Voxel> voxelMap;

static const int neighborDirs[26][3] = {
	{-1,-1,-1},{ 0,-1,-1},{ 1,-1,-1},
	{-1, 0,-1},{ 0, 0,-1},{ 1, 0,-1},
	{-1, 1,-1},{ 0, 1,-1},{ 1, 1,-1},
	{-1,-1, 0},{ 0,-1, 0},{ 1,-1, 0},
	{-1, 0, 0},           { 1, 0, 0},
	{-1, 1, 0},{ 0, 1, 0},{ 1, 1, 0},
	{-1,-1, 1},{ 0,-1, 1},{ 1,-1, 1},
	{-1, 0, 1},{ 0, 0, 1},{ 1, 0, 1},
	{-1, 1, 1},{ 0, 1, 1},{ 1, 1, 1}
};

void floodFill(const VoxelKey& seed, ui32 label)
{
	std::queue<VoxelKey> q;
	q.push(seed);

	auto it = voxelMap.find(seed);
	if (it == voxelMap.end() || it->second.label != invalid_ui32)
		return;

	it->second.label = label;

	while (!q.empty()) {
		VoxelKey cur = q.front();
		q.pop();

		for (auto& dir : neighborDirs) {
			VoxelKey nkey{ cur.x + dir[0], cur.y + dir[1], cur.z + dir[2] };

			auto nit = voxelMap.find(nkey);
			if (nit != voxelMap.end() && nit->second.label == invalid_ui32) {
				nit->second.label = label;
				q.push(nkey);
			}
		}
	}
}

// 전체 라벨링 함수
void ConnectedComponentLabeling()
{
	std::atomic<ui32> currentLabel{ 1 };

	std::vector<VoxelKey> allKeys;
	allKeys.reserve(voxelMap.size());
	for (auto& [key, _] : voxelMap)
		allKeys.push_back(key);

#pragma omp parallel for schedule(dynamic)
	for (int i = 0; i < (int)allKeys.size(); ++i) {
		const auto& key = allKeys[i];
		auto it = voxelMap.find(key);
		if (it == voxelMap.end()) continue;
		if (it->second.label != invalid_ui32) continue;

		ui32 label = currentLabel.fetch_add(1);

#pragma omp critical
		{
			floodFill(key, label);
		}
	}

	std::cout << "Total connected components: "
		<< (currentLabel.load() - 1) << std::endl;
}

std::vector<std::vector<ui32>> ClusterPointsByConnectivity(const std::vector<glm::vec3>& points)
{
	voxelMap.clear();

	// (1) point → voxel
	for (ui32 i = 0; i < points.size(); ++i) {
		VoxelKey key = VoxelKey::ToVoxelKey(points[i], voxelSize);
		voxelMap[key].pointIndices.push_back(i);
	}

	// (2) CCL
	ConnectedComponentLabeling();

	// (3) cluster group 생성
	std::unordered_map<ui32, std::vector<ui32>> clusters;
	for (auto& [key, v] : voxelMap) {
		if (v.label == 0) continue;
		clusters[v.label].insert(
			clusters[v.label].end(),
			v.pointIndices.begin(),
			v.pointIndices.end()
		);
	}

	std::vector<std::vector<ui32>> clusterList;
	clusterList.reserve(clusters.size());
	for (auto& [label, pts] : clusters)
		clusterList.push_back(std::move(pts));

	return clusterList;
}






int main(int argc, char** argv)
{
	std::cout << "AppFeather" << std::endl;

	Feather.Initialize(1920, 1080);
	Feather.SetConsoleWindowIndex(3);
	Feather.SetMainWindowIndex(2);

	GLuint VAO, VBO;

	auto w = Feather.GetFeatherWindow();

#pragma region AppMain
	{
		auto appMain = Feather.CreateEntity("AppMain");
		Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
			if (GLFW_KEY_ESCAPE == event.keyCode)
			{
				glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
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

				auto renderable = Feather.GetComponent<Renderable>(entity);
				if (event.button == 0 && event.action == 0)
				{
					auto ray = manipulator->GetCamera()->ScreenPointToRay(event.xpos, event.ypos, w->GetWidth(), w->GetHeight());
					//VD::Clear("PickingRay");
					VD::AddLine("PickingRay", ray.origin, ray.direction * 500.0f, { 1.0f, 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f });
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

		{
			PLYFormat ply;
			ply.Deserialize("D:\\Debug\\PLY\\input.ply");

			auto entity = Feather.CreateEntity("PointCloud");
			auto renderable = Feather.CreateComponent<Renderable>(entity);
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->AddShader(Feather.CreateShader("Instancing", File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs")));
			renderable->AddShader(Feather.CreateShader("InstancingWithoutNormal", File("../../res/Shaders/InstancingWithoutNormal.vs"), File("../../res/Shaders/InstancingWithoutNormal.fs")));
			renderable->SetActiveShaderIndex(1);

			auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere({ 0.0f, 0.0f, 0.0f }, 0.5f, 6, 6);
			//auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "half");
			renderable->AddIndices(indices);
			renderable->AddVertices(vertices);
			renderable->AddNormals(normals);
			renderable->AddColors(colors);
			renderable->AddUVs(uvs);

			AABB aabb{ {FLT_MAX, FLT_MAX, FLT_MAX}, {-FLT_MAX, -FLT_MAX, -FLT_MAX} };

			//for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
			//{
			//	auto x = ply.GetPoints()[3 * i + 0];
			//	auto y = ply.GetPoints()[3 * i + 1];
			//	auto z = ply.GetPoints()[3 * i + 2];
			//	glm::vec3 position = glm::vec3(x, y, z);

			//	aabb.min.x = fminf(aabb.min.x, x);
			//	aabb.min.y = fminf(aabb.min.y, y);
			//	aabb.min.z = fminf(aabb.min.z, z);

			//	aabb.max.x = fmaxf(aabb.max.x, x);
			//	aabb.max.y = fmaxf(aabb.max.y, y);
			//	aabb.max.z = fmaxf(aabb.max.z, z);

			//	auto nx = ply.GetNormals()[3 * i + 0];
			//	auto ny = ply.GetNormals()[3 * i + 1];
			//	auto nz = ply.GetNormals()[3 * i + 2];
			//	glm::vec3 normal = glm::vec3(nx, ny, nz);

			//	auto r = ply.GetColors()[4 * i + 0];
			//	auto g = ply.GetColors()[4 * i + 1];
			//	auto b = ply.GetColors()[4 * i + 2];
			//	auto a = ply.GetColors()[4 * i + 3];

			//	renderable->AddInstanceColor({r, g, b, 1.0f});
			//	renderable->AddInstanceNormal(normal);

			//	glm::mat4 tm = glm::identity<glm::mat4>();
			//	glm::mat4 rot = glm::mat4(1.0f);
			//	if (glm::length(normal) > 0.0001f)
			//	{
			//		glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(0, 0, 1), normal));
			//		float angle = acos(glm::dot(glm::normalize(normal), glm::vec3(0, 0, 1)));
			//		if (glm::length(axis) > 0.0001f)
			//			rot = glm::rotate(glm::mat4(1.0f), angle, axis);
			//	}

			//	tm = glm::translate(tm, position) * rot * glm::scale(glm::mat4(1.0f), glm::vec3(0.1f));

			//	renderable->AddInstanceTransform(tm);
			//	renderable->IncreaseNumberOfInstances();
			//}

			//Feather.CreateEventCallback<KeyEvent>(entity, [](Entity entity, const KeyEvent& event) {
			//	auto renderable = Feather.GetComponent<Renderable>(entity);
			//	if (nullptr == renderable) return;

			//	if (0 == event.action)
			//	{
			//		if (GLFW_KEY_GRAVE_ACCENT == event.keyCode)
			//		{
			//			renderable->NextDrawingMode();
			//		}
			//		else if (GLFW_KEY_1 == event.keyCode)
			//		{
			//			renderable->SetActiveShaderIndex(0);
			//		}
			//		else if (GLFW_KEY_2 == event.keyCode)
			//		{
			//			renderable->SetActiveShaderIndex(1);
			//		}
			//	}
			//	});

			//TS(Occupy);
			//for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
			//{
			//	auto x = ply.GetPoints()[3 * i + 0];
			//	auto y = ply.GetPoints()[3 * i + 1];
			//	auto z = ply.GetPoints()[3 * i + 2];
			//	glm::vec3 position = glm::vec3(x, y, z);

			//	voxelMap[{ (i32)floorf(x / voxelSize), (i32)floorf(y / voxelSize), (i32)floorf(z / voxelSize) }] = Voxel{ 0, (ui32)i };
			//}
			//TE(Occupy);

			//TS(CCL);
			//ConnectedComponentLabeling();
			//TE(CCL);

			std::unordered_map<VoxelKey, Voxel> tempVoxelMap;

			std::vector<glm::vec3> points;
			for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
			{
				auto x = ply.GetPoints()[3 * i + 0];
				auto y = ply.GetPoints()[3 * i + 1];
				auto z = ply.GetPoints()[3 * i + 2];
				glm::vec3 position = glm::vec3(x, y, z);
				//points.push_back(position);

				VoxelKey key = VoxelKey::ToVoxelKey(position, voxelSize);
				tempVoxelMap[key].pointIndices.push_back(i);
			}

			for (auto& kvp : tempVoxelMap)
			{
				auto& key = kvp.first;
				auto& p = VoxelKey::FromVoxelKey(key, voxelSize);
				points.push_back(p);
			}

			TS(Clustering);
			auto clusters = ClusterPointsByConnectivity(points);
			TE(Clustering);

			std::cout << "Cluster count: " << clusters.size() << "\n";
			for (size_t i = 0; i < clusters.size(); ++i)
				std::cout << "Cluster " << i << " points: " << clusters[i].size() << "\n";

			auto clusterColors = Color::GetContrastingColors(clusters.size() + 10);
			for (size_t i = 0; i < clusters.size(); i++)
			{
				auto& cluster = clusters[i];
				auto clusterColor = clusterColors[i % clusterColors.size()];

				for (auto& index : cluster)
				{
					VD::AddSphere("Clusters", points[index], voxelSize * 0.5f, clusterColor);
				}
			}
		}
		});

	Feather.Run();

	Feather.Terminate();

	return 0;
}
