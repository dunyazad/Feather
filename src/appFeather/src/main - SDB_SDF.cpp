//#include <libFeather.h>
//
//using VD = VisualDebugging;
//using namespace libRxTx;
//
//struct SortEntry
//{
//	uint64_t mortonCode;
//	size_t originalIndex;
//
//	// 정렬을 위한 비교 연산자
//	bool operator<(const SortEntry& other) const
//	{
//		return mortonCode < other.mortonCode;
//	}
//};
//
//void SortDataByMorton(
//	std::vector<glm::vec3>& points,
//	std::vector<glm::vec3>& normals,
//	std::vector<glm::vec3>& colors,
//	float voxelSize,
//	int vpB) // VpB: Voxel per Block (예: 8)
//{
//	size_t numPoints = points.size() / 3;
//	if (numPoints == 0) return;
//
//	// 1. 전체 포인트의 AABB(최소 좌표) 계산
//	// Morton Code의 기준점(Origin)을 잡기 위함입니다.
//	glm::vec3 minP(std::numeric_limits<float>::max());
//
//	for (size_t i = 0; i < numPoints; ++i)
//	{
//		if (points[i].x < minP.x) minP.x = points[i].x;
//		if (points[i].y < minP.y) minP.y = points[i].y;
//		if (points[i].z < minP.z) minP.z = points[i].z;
//	}
//
//	// 2. 정렬 키(Morton Code) 생성
//	std::vector<SortEntry> entries(numPoints);
//	float blockSize = voxelSize * vpB;
//
//	// AABB Min을 기준으로 약간의 여유를 두고 Origin 설정 (음수 좌표 방지 등)
//	glm::vec3 origin = minP - glm::vec3(blockSize * 2.0f);
//
//	for (size_t i = 0; i < numPoints; ++i)
//	{
//		// 제공해주신 Morton3D 클래스 사용
//		entries[i].mortonCode = Morton3D::EncodeFromVec3(points[i], origin, blockSize);
//		entries[i].originalIndex = i;
//	}
//
//	// 3. 정렬 수행 (오름차순)
//	std::sort(entries.begin(), entries.end());
//
//	// 4. 데이터 재배열 (Reordering)
//	std::vector<glm::vec3> sortedPoints(points.size());
//	std::vector<glm::vec3> sortedNormals(points.size());
//	std::vector<glm::vec3> sortedColors(points.size());
//
//	bool hasNormals = !normals.empty();
//	bool hasColors = !colors.empty();
//
//	if (hasNormals) sortedNormals.resize(normals.size());
//	if (hasColors) sortedColors.resize(colors.size());
//
//	for (size_t i = 0; i < numPoints; ++i)
//	{
//		size_t oldIdx = entries[i].originalIndex;
//
//		// Point 복사
//		sortedPoints[i] = points[oldIdx];
//
//		// Normal 복사
//		if (hasNormals)
//		{
//			sortedNormals[i] = normals[oldIdx];
//		}
//
//		// Color 복사
//		if (hasColors)
//		{
//			sortedColors[i] = colors[oldIdx];
//		}
//	}
//
//	// 5. 원본 데이터 교체
//	points = std::move(sortedPoints);
//	if (hasNormals) normals = std::move(sortedNormals);
//	if (hasColors) colors = std::move(sortedColors);
//}
//
//void SortDataByMorton(
//	std::vector<float>& points,
//	std::vector<float>& normals,
//	std::vector<float>& colors,
//	float voxelSize,
//	int vpB) // VpB: Voxel per Block (예: 8)
//{
//	size_t numPoints = points.size() / 3;
//	if (numPoints == 0) return;
//
//	// 1. 전체 포인트의 AABB(최소 좌표) 계산
//	// Morton Code의 기준점(Origin)을 잡기 위함입니다.
//	glm::vec3 minP(std::numeric_limits<float>::max());
//
//	for (size_t i = 0; i < numPoints; ++i)
//	{
//		float x = points[i * 3 + 0];
//		float y = points[i * 3 + 1];
//		float z = points[i * 3 + 2];
//
//		if (x < minP.x) minP.x = x;
//		if (y < minP.y) minP.y = y;
//		if (z < minP.z) minP.z = z;
//	}
//
//	// 2. 정렬 키(Morton Code) 생성
//	std::vector<SortEntry> entries(numPoints);
//	float blockSize = voxelSize * vpB;
//
//	// AABB Min을 기준으로 약간의 여유를 두고 Origin 설정 (음수 좌표 방지 등)
//	glm::vec3 origin = minP - glm::vec3(blockSize * 2.0f);
//
//	for (size_t i = 0; i < numPoints; ++i)
//	{
//		glm::vec3 p(
//			points[i * 3 + 0],
//			points[i * 3 + 1],
//			points[i * 3 + 2]
//		);
//
//		// 제공해주신 Morton3D 클래스 사용
//		entries[i].mortonCode = Morton3D::EncodeFromVec3(p, origin, blockSize);
//		entries[i].originalIndex = i;
//	}
//
//	// 3. 정렬 수행 (오름차순)
//	std::sort(entries.begin(), entries.end());
//
//	// 4. 데이터 재배열 (Reordering)
//	std::vector<float> sortedPoints(points.size());
//	std::vector<float> sortedNormals;
//	std::vector<float> sortedColors;
//
//	bool hasNormals = !normals.empty();
//	bool hasColors = !colors.empty();
//
//	if (hasNormals) sortedNormals.resize(normals.size());
//	if (hasColors) sortedColors.resize(colors.size());
//
//	for (size_t i = 0; i < numPoints; ++i)
//	{
//		size_t oldIdx = entries[i].originalIndex;
//
//		// Point 복사
//		sortedPoints[i * 3 + 0] = points[oldIdx * 3 + 0];
//		sortedPoints[i * 3 + 1] = points[oldIdx * 3 + 1];
//		sortedPoints[i * 3 + 2] = points[oldIdx * 3 + 2];
//
//		// Normal 복사
//		if (hasNormals)
//		{
//			sortedNormals[i * 3 + 0] = normals[oldIdx * 3 + 0];
//			sortedNormals[i * 3 + 1] = normals[oldIdx * 3 + 1];
//			sortedNormals[i * 3 + 2] = normals[oldIdx * 3 + 2];
//		}
//
//		// Color 복사
//		if (hasColors)
//		{
//			sortedColors[i * 3 + 0] = colors[oldIdx * 3 + 0];
//			sortedColors[i * 3 + 1] = colors[oldIdx * 3 + 1];
//			sortedColors[i * 3 + 2] = colors[oldIdx * 3 + 2];
//		}
//	}
//
//	// 5. 원본 데이터 교체
//	points = std::move(sortedPoints);
//	if (hasNormals) normals = std::move(sortedNormals);
//	if (hasColors) colors = std::move(sortedColors);
//}
//
//const int VpB = 8;
//const int VpBHalf = 4;
//
//struct Voxel
//{
//	bool valid = false;
//	float signedDistance = FLT_MAX;
//	float weight = 0.0f;
//	glm::vec3 normal = glm::zero<glm::vec3>();
//	glm::vec3 color = glm::zero<glm::vec3>();
//};
//
//struct DataBlock
//{
//	glm::vec3 blockMin = glm::zero<glm::vec3>();
//	Voxel voxels[VpB * VpB * VpB];
//
//	void Initialize()
//	{
//		memset(voxels, 0, sizeof(Voxel) * VpB * VpB * VpB);
//	}
//
//	Voxel* FindVoxel(const glm::vec3& position, float voxelSize)
//	{
//		const float blockSize = voxelSize * VpB;
//		const float epsilon = 1e-4f; // 부동 소수점 오차 허용 범위
//
//		glm::vec3 delta = position - blockMin;
//
//		// half-open interval: 0 <= delta < blockSize
//		// Epsilon을 사용하여 경계에 걸친 점들을 포용합니다.
//		if (delta.x < -epsilon || delta.x >= blockSize + epsilon ||
//			delta.y < -epsilon || delta.y >= blockSize + epsilon ||
//			delta.z < -epsilon || delta.z >= blockSize + epsilon)
//		{
//			return nullptr;
//		}
//
//		int xIndex = (int)floorf(delta.x / voxelSize);
//		int yIndex = (int)floorf(delta.y / voxelSize);
//		int zIndex = (int)floorf(delta.z / voxelSize);
//
//		// 오차로 인해 인덱스가 -1이 되거나 VpB가 되는 경우를 방지하기 위해 클램핑(Clamping) 처리
//		if (xIndex < 0) xIndex = 0;
//		if (xIndex >= VpB) xIndex = VpB - 1;
//
//		if (yIndex < 0) yIndex = 0;
//		if (yIndex >= VpB) yIndex = VpB - 1;
//
//		if (zIndex < 0) zIndex = 0;
//		if (zIndex >= VpB) zIndex = VpB - 1;
//
//		return &voxels[zIndex * VpB * VpB + yIndex * VpB + xIndex];
//	}
//};
//
//typedef uint64_t DataBlockKey;
//
//struct SparseDataBlock
//{
//	float voxelSize = 0.1f;
//	glm::vec3 volumeMin = glm::zero<glm::vec3>();
//
//	std::unordered_map<DataBlockKey, DataBlock> dataBlocks;
//
//	DataBlock* CreateDataBlock(DataBlockKey key)
//	{
//		auto it = dataBlocks.find(key);
//		if (it != dataBlocks.end())
//		{
//			return &((*it).second);
//		}
//		else
//		{
//			uint32_t bx, by, bz;
//			Morton3D::Decode(key, bx, by, bz);
//			dataBlocks[key].blockMin = volumeMin +
//				glm::vec3(
//					(float)bx * voxelSize * VpB,
//					(float)by * voxelSize * VpB,
//					(float)bz * voxelSize * VpB
//				);
//			return &dataBlocks[key];
//		}
//	}
//
//	DataBlock* FindDataBlock(const glm::vec3& position)
//	{
//		auto key = Morton3D::EncodeFromVec3(position, volumeMin, voxelSize * VpB);
//
//		return FindDataBlock(key);
//	}
//
//	DataBlock* FindDataBlock(DataBlockKey key)
//	{
//		auto it = dataBlocks.find(key);
//		if (it != dataBlocks.end())
//		{
//			return &((*it).second);
//		}
//		else
//		{
//			return nullptr;
//		}
//	}
//
//	void FromPLY(const std::string& filename)
//	{
//		TS(PLYLoading);
//		PLYFormat ply;
//		ply.Deserialize(filename);
//		TE(PLYLoading);
//
//		auto [minx, miny, minz] = ply.GetAABBMin();
//		glm::vec3 aabbMin(minx, miny, minz);
//
//		float blockSize = voxelSize * VpB;
//
//		volumeMin.x = std::floor(aabbMin.x / blockSize) * blockSize;
//		volumeMin.y = std::floor(aabbMin.y / blockSize) * blockSize;
//		volumeMin.z = std::floor(aabbMin.z / blockSize) * blockSize;
//
//		TS(DataCopy);
//		std::vector<glm::vec3> positions(ply.GetPoints().size() / 3);
//		memcpy(positions.data(), ply.GetPoints().data(), sizeof(float) * ply.GetPoints().size());
//		std::vector<glm::vec3> normals(ply.GetPoints().size() / 3);
//		memcpy(normals.data(), ply.GetNormals().data(), sizeof(float) * ply.GetPoints().size());
//		std::vector<glm::vec3> colors(ply.GetPoints().size() / 3);
//		memcpy(colors.data(), ply.GetColors().data(), sizeof(float) * ply.GetPoints().size());
//		TE(DataCopy);
//
//		TS(Sorting);
//		SortDataByMorton(positions, normals, colors, voxelSize, VpB);
//		TE(Sorting);
//
//		FromPoints(
//			(glm::vec3*)ply.GetPoints().data(),
//			(glm::vec3*)ply.GetNormals().data(),
//			(glm::vec3*)ply.GetColors().data(),
//			ply.GetPoints().size() / 3,
//			{ minx, miny, minz });
//	}
//
//	void FromPoints(glm::vec3* points, glm::vec3* normals, glm::vec3* colors, unsigned int numberOfPoints, const glm::vec3& aabbMin)
//	{
//		float blockSize = voxelSize * VpB;
//		float offset = 3.0f * voxelSize;
//
//		// [1] volumeMin 계산 (이전과 동일)
//		volumeMin.x = std::floor((aabbMin.x - offset) / blockSize) * blockSize;
//		volumeMin.y = std::floor((aabbMin.y - offset) / blockSize) * blockSize;
//		volumeMin.z = std::floor((aabbMin.z - offset) / blockSize) * blockSize;
//
//		int offsetSteps = (int)std::ceil(offset / voxelSize);
//
//		TS(Occupy);
//
//		// [최적화] VpB가 2의 거듭제곱(8)이므로 비트 연산 활용
//		// 8 = 2^3
//		const int VpB_Shift = 3;
//		const int VpB_Mask = VpB - 1; // 7 (0b111)
//
//		// 캐싱 변수: Morton Key 대신 '블록 좌표(Index)'를 직접 비교
//		glm::ivec3 lastBlockIdx = glm::ivec3(INT_MAX);
//		DataBlock* lastBlock = nullptr;
//
//		// 반복문 밖에서 상수 계산
//		float invVoxelSize = 1.0f / voxelSize;
//
//		for (size_t i = 0; i < numberOfPoints; i++)
//		{
//			glm::vec3 position = points[i];
//
//			// Global Voxel Index 계산 (float 나눗셈 1회)
//			int cx = (int)std::floor((position.x - volumeMin.x) * invVoxelSize);
//			int cy = (int)std::floor((position.y - volumeMin.y) * invVoxelSize);
//			int cz = (int)std::floor((position.z - volumeMin.z) * invVoxelSize);
//
//			// 범위 루프
//			for (int z = cz - offsetSteps; z <= cz + offsetSteps; z++)
//			{
//				for (int y = cy - offsetSteps; y <= cy + offsetSteps; y++)
//				{
//					for (int x = cx - offsetSteps; x <= cx + offsetSteps; x++)
//					{
//						// [Core Optimization]
//						// 전역 복셀 인덱스(Global Index)를 통해 블록 인덱스와 로컬 인덱스를 분리
//
//						// 1. 블록 좌표 계산 (비트 시프트) :: / 8 과 동일
//						int bx = x >> VpB_Shift;
//						int by = y >> VpB_Shift;
//						int bz = z >> VpB_Shift;
//
//						// 2. 블록 좌표가 변했는지 확인 (매우 빠른 정수 비교)
//						if (bx != lastBlockIdx.x || by != lastBlockIdx.y || bz != lastBlockIdx.z)
//						{
//							// 블록이 바뀌었을 때만 Morton Key 계산 및 맵 검색
//							// 주의: Encode는 음수 좌표 처리를 위해 좌표 보정이 필요할 수 있으나, 
//							// volumeMin을 넉넉히 잡았으므로 bx, by, bz는 0 이상이라 가정합니다.
//
//							// 블록의 코너 좌표(월드 공간)가 필요하다면:
//							// float blockWorldX = volumeMin.x + (float)bx * blockSize; ...
//							// 하지만 여기서는 Key 생성을 위해 단순히 Morton Encode만 하면 됩니다.
//
//							auto key = Morton3D::Encode(bx, by, bz);
//
//							lastBlock = FindDataBlock(key);
//							if (nullptr == lastBlock)
//							{
//								lastBlock = CreateDataBlock(key);
//							}
//
//							lastBlockIdx = { bx, by, bz };
//						}
//
//						// 3. 로컬 인덱스 계산 (비트 마스킹) :: % 8 과 동일
//						int lx = x & VpB_Mask;
//						int ly = y & VpB_Mask;
//						int lz = z & VpB_Mask;
//
//						// 4. 복셀 접근 (FindVoxel 함수 호출 없이 배열 직접 접근)
//						// DataBlock 구조체 내의 voxels 배열은 1차원이므로 인덱스 계산 필요
//						// voxels[z * 64 + y * 8 + x]
//						// 64 = VpB * VpB = 1 << (3+3) = 1 << 6
//						// 8  = VpB       = 1 << 3
//
//						if (lastBlock)
//						{
//							int voxelIdx = (lz << (VpB_Shift * 2)) | (ly << VpB_Shift) | lx;
//							lastBlock->voxels[voxelIdx].valid = true;
//						}
//					}
//				}
//			}
//		}
//		TE(Occupy);
//	}
//
//	void ShowBlocks()
//	{
//		for (auto& [key, block] : dataBlocks)
//		{
//			auto center = block.blockMin + glm::vec3(voxelSize * VpBHalf, voxelSize * VpBHalf, voxelSize * VpBHalf);
//			VD::AddWiredBox("datablock", center, glm::vec3(voxelSize * VpB, voxelSize * VpB, voxelSize * VpB), Color::red());
//
//			for (size_t z = 0; z < VpB; z++)
//			{
//				for (size_t y = 0; y < VpB; y++)
//				{
//					for (size_t x = 0; x < VpB; x++)
//					{
//						auto voxel = block.voxels[z * VpB * VpB + y * VpB + x];
//						if (voxel.valid)
//						{
//							auto vcenter = block.blockMin
//								+ glm::vec3((float)x * voxelSize, (float)y * voxelSize, (float)z * voxelSize)
//								+ glm::vec3(0.5f * voxelSize, 0.5f * voxelSize, 0.5f * voxelSize);
//
//							VD::AddWiredBox("datablock", vcenter, glm::vec3(voxelSize, voxelSize, voxelSize), Color::yellow());
//						}
//					}
//				}
//			}
//		}
//	}
//};
//
//
//
//
//
//
//
//
//
//
//static inline std::string FormatWithCommas(size_t value)
//{
//	std::string numStr = std::to_string(value);
//	int insertPosition = static_cast<int>(numStr.length()) - 3;
//	while (insertPosition > 0)
//	{
//		numStr.insert(insertPosition, ",");
//		insertPosition -= 3;
//	}
//	return numStr;
//}
//
//// --- Helper Functions ---
//static inline void DepthToColor(float d, unsigned char& r, unsigned char& g, unsigned char& b)
//{
//	d = std::clamp(d, 0.0f, 1.0f);
//	float r_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.75f)), 0.0f, 1.0f);
//	float g_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.50f)), 0.0f, 1.0f);
//	float b_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.25f)), 0.0f, 1.0f);
//	r = static_cast<unsigned char>(r_f * 255.0f);
//	g = static_cast<unsigned char>(g_f * 255.0f);
//	b = static_cast<unsigned char>(b_f * 255.0f);
//}
//
//int main(int argc, char** argv)
//{
//	std::cout << "AppFeather" << std::endl;
//
//	Feather.Initialize(1920, 1080);
//	Feather.SetConsoleWindowIndex(3);
//	Feather.SetMainWindowIndex(2);
//
//	auto w = Feather.GetFeatherWindow();
//
//#pragma region AppMain
//	{
//		auto appMain = Feather.CreateEntity("AppMain");
//		Feather.CreateEventCallback<KeyEvent>(appMain, [](Entity entity, const KeyEvent& event) {
//			if (GLFW_KEY_ESCAPE == event.keyCode)
//			{
//				glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
//			}
//			else if (GLFW_KEY_SPACE == event.keyCode)
//			{
//				if (event.action == 0)
//				{
//					Feather.GetImmediateModeRenderSystem()->ToggleEnable();
//				}
//			}
//			else if (GLFW_KEY_ENTER == event.keyCode && event.action == 0)
//			{
//				glBindFramebuffer(GL_FRAMEBUFFER, 0);
//
//				auto w = Feather.GetFeatherWindow();
//
//				int width, height;
//				glfwGetFramebufferSize(w->GetGLFWwindow(), &width, &height);
//
//				std::vector<float> depth(width * height);
//				glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data());
//
//				// Flip Y
//				std::vector<float> flipped(width * height);
//				for (int y = 0; y < height; ++y)
//				{
//					memcpy(&flipped[y * width],
//						&depth[(height - 1 - y) * width],
//						width * sizeof(float));
//				}
//
//				// 깊이값 최소/최대 구하기
//				float minD = 1.0f, maxD = 0.0f;
//				for (float v : flipped)
//				{
//					if (v < minD) minD = v;
//					if (v > maxD) maxD = v;
//				}
//				printf("[Depth] min=%.6f, max=%.6f, range=%.6f\n", minD, maxD, maxD - minD);
//				float range = std::max(maxD - minD, 1e-6f);
//
//				// RAW 저장
//				FILE* f = fopen("depth.raw", "wb");
//				fwrite(flipped.data(), sizeof(float), width * height, f);
//				fclose(f);
//
//				// pseudo-color PNG
//				std::vector<unsigned char> rgb(width * height * 3);
//				for (int i = 0; i < width * height; ++i)
//				{
//					float norm = (flipped[i] - minD) / range;
//					unsigned char r, g, b;
//					DepthToColor(norm, r, g, b);
//					rgb[i * 3 + 0] = r;
//					rgb[i * 3 + 1] = g;
//					rgb[i * 3 + 2] = b;
//				}
//
//				stbi_write_png("depth_color.png", width, height, 3, rgb.data(), width * 3);
//				printf("[DepthMap] Saved to depth_color.png\n");
//			}
//			});
//	}
//#pragma endregion
//
//#pragma region Camera
//	{
//		Entity cam = Feather.CreateEntity("Camera");
//		auto pcam = Feather.CreateComponent<PerspectiveCamera>(cam);
//		auto pcamMan = Feather.CreateComponent<CameraManipulatorTrackball>(cam);
//		pcamMan->SetCamera(pcam);
//
//		Feather.CreateEventCallback<FrameBufferResizeEvent>(cam, [pcam](Entity entity, const FrameBufferResizeEvent& event) {
//			auto window = Feather.GetFeatherWindow();
//			auto aspectRatio = (f32)window->GetWidth() / (f32)window->GetHeight();
//			pcam->SetAspectRatio(aspectRatio);
//			});
//
//		Feather.CreateEventCallback<KeyEvent>(cam, [](Entity entity, const KeyEvent& event) {
//			Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnKey(event);
//			});
//
//		Feather.CreateEventCallback<MousePositionEvent>(cam, [](Entity entity, const MousePositionEvent& event) {
//			Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMousePosition(event);
//			});
//
//		Feather.CreateEventCallback<MouseButtonEvent>(cam, [&](Entity entity, const MouseButtonEvent& event) {
//			auto manipulator = Feather.GetComponent<CameraManipulatorTrackball>(entity);
//			manipulator->OnMouseButton(event);
//
//			if (event.button == 0 && event.action == 0)
//			{
//				int fbWidth, fbHeight;
//				glfwGetFramebufferSize(w->GetGLFWwindow(), &fbWidth, &fbHeight);
//
//				int readX = static_cast<int>(event.xpos);
//				int readY = fbHeight - static_cast<int>(event.ypos) - 1;
//
//				float depth = 0.0f;
//				glReadPixels(readX, readY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
//				printf("Depth: %f\n", depth);
//			}
//			});
//
//		Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) {
//			Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event);
//			});
//	}
//#pragma endregion
//
//	Feather.AddOnInitializeCallback([&]() {
//		{
//			PLYFormat ply;
//			ply.Deserialize("D:\\Debug\\PLY\\inputA.ply");
//			auto [minx, miny, minz] = ply.GetAABBMin();
//
//			for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
//			{
//				auto x = ply.GetPoints()[i * 3 + 0];
//				auto y = ply.GetPoints()[i * 3 + 1];
//				auto z = ply.GetPoints()[i * 3 + 2];
//
//				VD::AddSphere("point", {x, y, z}, 0.02f, Color::white());
//			}
//
//			SparseDataBlock sdb;
//			//sdb.FromPLY("D:\\Debug\\PLY\\inputA.ply");
//			TS(Sorting);
//			// voxelSize와 VpB는 SparseDataBlock에서 사용하는 값과 동일하게 넣어줍니다.
//			// 예: voxelSize = 0.1f, VpB = 8
//			SortDataByMorton(
//				ply.GetPoints(),
//				ply.GetNormals(),
//				ply.GetColors(),
//				0.1f, // SparseDataBlock::voxelSize
//				8     // VpB
//			);
//			TE(Sorting);
//
//			// 정렬된 데이터를 사용하여 Sparse Voxel 생성
//			// 정렬 덕분에 FromPoints 내부의 'lastBlock' 캐시 적중률이 극대화됩니다.
//			sdb.FromPoints(
//				(glm::vec3*)ply.GetPoints().data(),
//				(glm::vec3*)ply.GetNormals().data(),
//				(glm::vec3*)ply.GetColors().data(),
//				ply.GetPoints().size() / 3,
//				{ minx, miny, minz } // min값은 PLY 로딩 시 구한 값 사용
//			);
//
//			sdb.ShowBlocks();
//
//			alog("Total DataBlocks : %s\n", FormatWithCommas(sdb.dataBlocks.size()).c_str());
//			alog("DataBlock Size : %zd\n", sizeof(DataBlock));
//			alog("Total Memory : %s bytes\n", FormatWithCommas(sdb.dataBlocks.size() * sizeof(DataBlock)).c_str());
//		}
//
//#pragma region Status Panel
//		{
//			auto gui = Feather.GetRegistry().create();
//			auto statusPanel = Feather.GetRegistry().emplace<StatusPanel>(gui);
//
//			Feather.CreateEventCallback<MousePositionEvent>(gui, [](Entity entity, const MousePositionEvent& event) {
//				auto& component = Feather.GetRegistry().get<StatusPanel>(entity);
//				component.mouseX = event.xpos;
//				component.mouseY = event.ypos;
//				});
//		}
//#pragma endregion
//
//		});
//
//	Feather.Run();
//	Feather.Terminate();
//
//	return 0;
//}


















#include <libFeather.h>

using VD = VisualDebugging;
using namespace libRxTx;

// --- Morton Sorting Helper ---
struct SortEntry
{
	uint64_t mortonCode;
	size_t originalIndex;

	bool operator<(const SortEntry& other) const
	{
		return mortonCode < other.mortonCode;
	}
};

// float 버전 (PLY 데이터용)
void SortDataByMorton(
	std::vector<float>& points,
	std::vector<float>& normals,
	std::vector<float>& colors,
	float voxelSize,
	int vpB)
{
	size_t numPoints = points.size() / 3;
	if (numPoints == 0) return;

	glm::vec3 minP(std::numeric_limits<float>::max());
	for (size_t i = 0; i < numPoints; ++i)
	{
		float x = points[i * 3 + 0];
		float y = points[i * 3 + 1];
		float z = points[i * 3 + 2];
		if (x < minP.x) minP.x = x;
		if (y < minP.y) minP.y = y;
		if (z < minP.z) minP.z = z;
	}

	std::vector<SortEntry> entries(numPoints);
	float blockSize = voxelSize * vpB;
	glm::vec3 origin = minP - glm::vec3(blockSize * 2.0f);

	for (size_t i = 0; i < numPoints; ++i)
	{
		glm::vec3 p(points[i * 3 + 0], points[i * 3 + 1], points[i * 3 + 2]);
		entries[i].mortonCode = Morton3D::EncodeFromVec3(p, origin, blockSize);
		entries[i].originalIndex = i;
	}

	std::sort(entries.begin(), entries.end());

	std::vector<float> sortedPoints(points.size());
	std::vector<float> sortedNormals;
	std::vector<float> sortedColors;

	bool hasNormals = !normals.empty();
	bool hasColors = !colors.empty();

	if (hasNormals) sortedNormals.resize(normals.size());
	if (hasColors) sortedColors.resize(colors.size());

	for (size_t i = 0; i < numPoints; ++i)
	{
		size_t oldIdx = entries[i].originalIndex;
		
		sortedPoints[i * 3 + 0] = points[oldIdx * 3 + 0];
		sortedPoints[i * 3 + 1] = points[oldIdx * 3 + 1];
		sortedPoints[i * 3 + 2] = points[oldIdx * 3 + 2];

		if (hasNormals)
		{
			sortedNormals[i * 3 + 0] = normals[oldIdx * 3 + 0];
			sortedNormals[i * 3 + 1] = normals[oldIdx * 3 + 1];
			sortedNormals[i * 3 + 2] = normals[oldIdx * 3 + 2];
		}
		if (hasColors)
		{
			sortedColors[i * 3 + 0] = colors[oldIdx * 3 + 0];
			sortedColors[i * 3 + 1] = colors[oldIdx * 3 + 1];
			sortedColors[i * 3 + 2] = colors[oldIdx * 3 + 2];
		}
	}

	points = std::move(sortedPoints);
	if (hasNormals) normals = std::move(sortedNormals);
	if (hasColors) colors = std::move(sortedColors);
}

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

	std::atomic_flag lock = ATOMIC_FLAG_INIT;

	void Initialize()
	{
		memset(voxels, 0, sizeof(Voxel) * VpB * VpB * VpB);
		lock.clear();
	}

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

		if (xIndex < 0) xIndex = 0; if (xIndex >= VpB) xIndex = VpB - 1;
		if (yIndex < 0) yIndex = 0; if (yIndex >= VpB) yIndex = VpB - 1;
		if (zIndex < 0) zIndex = 0; if (zIndex >= VpB) zIndex = VpB - 1;

		return &voxels[zIndex * VpB * VpB + yIndex * VpB + xIndex];
	}

	// Thread-safe update using running average
	void UpdateVoxelSafe(int idx, float dist, const glm::vec3& inColor, const glm::vec3& inNormal, bool hasColor, bool hasNormal)
	{
		Voxel& v = voxels[idx];

		if (!v.valid)
		{
			v.valid = true;
			v.weight = 1.0f;
			v.signedDistance = dist; // 초기값 설정

			if (hasColor) v.color = inColor;
			if (hasNormal) v.normal = inNormal;
		}
		else
		{
			// 가중치 평균 (Weighted Running Average)
			// TSDF(Truncated Signed Distance Field) 통합 공식과 유사합니다.
			float newWeight = v.weight + 1.0f;
			float ratio = 1.0f / newWeight;

			// SDF Blending
			// 기존 거리값과 새로운 거리값을 가중 평균냅니다.
			v.signedDistance = (v.signedDistance * v.weight + dist) * ratio;

			// Color Blending
			if (hasColor)
			{
				v.color = glm::mix(v.color, inColor, ratio);
			}

			// Normal Blending
			if (hasNormal)
			{
				v.normal = glm::mix(v.normal, inNormal, ratio);
			}

			v.weight = newWeight;
		}
	}
};

typedef uint64_t DataBlockKey;

struct SparseDataBlock
{
	float voxelSize = 0.2f;
	glm::vec3 volumeMin = glm::zero<glm::vec3>();

	std::unordered_map<DataBlockKey, DataBlock> dataBlocks;

	DataBlock* CreateDataBlock(DataBlockKey key)
	{
		auto it = dataBlocks.find(key);
		if (it != dataBlocks.end())
		{
			return &((*it).second);
		}
		else
		{
			uint32_t bx, by, bz;
			Morton3D::Decode(key, bx, by, bz);
			dataBlocks[key].blockMin = volumeMin +
				glm::vec3(
					(float)bx * voxelSize * VpB,
					(float)by * voxelSize * VpB,
					(float)bz * voxelSize * VpB
				);
			return &dataBlocks[key];
		}
	}

	DataBlock* FindDataBlock(DataBlockKey key)
	{
		auto it = dataBlocks.find(key);
		if (it != dataBlocks.end())
		{
			return &((*it).second);
		}
		return nullptr;
	}

	/*
	void FromPoints(glm::vec3* points, glm::vec3* normals, glm::vec3* colors, unsigned int numberOfPoints, const glm::vec3& aabbMin)
	{
		float blockSize = voxelSize * VpB;
		float offset = 3.0f * voxelSize;

		// 1. volumeMin 설정 (offset 포함 안전 영역 확보)
		volumeMin.x = std::floor((aabbMin.x - offset) / blockSize) * blockSize;
		volumeMin.y = std::floor((aabbMin.y - offset) / blockSize) * blockSize;
		volumeMin.z = std::floor((aabbMin.z - offset) / blockSize) * blockSize;

		int offsetSteps = (int)std::ceil(offset / voxelSize);

		TS(Occupy);

		// [최적화 상수] VpB가 8(2^3)이므로 비트 연산 활용
		const int VpB_Shift = 3;
		const int VpB_Mask = VpB - 1; // 7 (0b111)

		// 캐싱 변수: Morton Key 대신 '블록 정수 좌표' 직접 비교
		glm::ivec3 lastBlockIdx = glm::ivec3(INT_MAX);
		DataBlock* lastBlock = nullptr;

		float invVoxelSize = 1.0f / voxelSize;

		for (size_t i = 0; i < numberOfPoints; i++)
		{
			glm::vec3 position = points[i];

			// Global Voxel Index 계산 (float 나눗셈 최소화)
			int cx = (int)std::floor((position.x - volumeMin.x) * invVoxelSize);
			int cy = (int)std::floor((position.y - volumeMin.y) * invVoxelSize);
			int cz = (int)std::floor((position.z - volumeMin.z) * invVoxelSize);

			// 범위 루프 (정수 인덱스)
			for (int z = cz - offsetSteps; z <= cz + offsetSteps; z++)
			{
				for (int y = cy - offsetSteps; y <= cy + offsetSteps; y++)
				{
					for (int x = cx - offsetSteps; x <= cx + offsetSteps; x++)
					{
						// [Core Optimization] 
						// 전역 인덱스 -> 블록 인덱스(bx,by,bz)와 로컬 인덱스(lx,ly,lz) 분리

						// 1. 블록 좌표 계산 (비트 시프트: / 8)
						int bx = x >> VpB_Shift;
						int by = y >> VpB_Shift;
						int bz = z >> VpB_Shift;

						// 2. 블록 변경 확인 (매우 빠른 정수 비교)
						if (bx != lastBlockIdx.x || by != lastBlockIdx.y || bz != lastBlockIdx.z)
						{
							// 블록이 바뀌었을 때만 해시맵 검색
							auto key = Morton3D::Encode(bx, by, bz);

							lastBlock = FindDataBlock(key);
							if (nullptr == lastBlock)
							{
								lastBlock = CreateDataBlock(key);
							}

							lastBlockIdx = { bx, by, bz };
						}

						// 3. 로컬 인덱스 계산 (비트 마스킹: % 8)
						int lx = x & VpB_Mask;
						int ly = y & VpB_Mask;
						int lz = z & VpB_Mask;

						// 4. 복셀 직접 접근 (FindVoxel 호출 제거)
						// 인덱스: z * 64 + y * 8 + x (VpB=8 기준)
						if (lastBlock)
						{
							int voxelIdx = (lz << (VpB_Shift * 2)) | (ly << VpB_Shift) | lx;
							lastBlock->voxels[voxelIdx].valid = true;
						}
					}
				}
			}
		}
		TE(Occupy);
	}
	*/

	DataBlock* GetBlockUnsafe(DataBlockKey key)
	{
		auto it = dataBlocks.find(key);
		if (it != dataBlocks.end()) return &it->second;
		return nullptr;
	}

	DataBlock* GetBlockReadonly(DataBlockKey key)
	{
		auto it = dataBlocks.find(key);
		if (it != dataBlocks.end()) return &it->second;
		return nullptr;
	}

	void FromPoints(glm::vec3* points, glm::vec3* normals, glm::vec3* colors, unsigned int numberOfPoints, const glm::vec3& aabbMin)
	{
		float blockSize = voxelSize * VpB;
		float offset = 3.0f * voxelSize;

		volumeMin.x = std::floor((aabbMin.x - offset) / blockSize) * blockSize;
		volumeMin.y = std::floor((aabbMin.y - offset) / blockSize) * blockSize;
		volumeMin.z = std::floor((aabbMin.z - offset) / blockSize) * blockSize;

		float invVoxelSize = 1.0f / voxelSize;
		int iOffset = (int)std::ceil(offset / voxelSize);

		bool hasNormals = (normals != nullptr);
		bool hasColors = (colors != nullptr);

		// ----------------------------------------------------------------
		// [Step 1] Point -> Unique Voxel Keys (For Block Creation)
		// ----------------------------------------------------------------
		TS(DedupVoxels);
		int maxThreads = omp_get_max_threads();
		std::vector<std::vector<uint64_t>> threadVoxelKeys(maxThreads);

#pragma omp parallel
		{
			int tid = omp_get_thread_num();
			threadVoxelKeys[tid].reserve(numberOfPoints / maxThreads);

#pragma omp for schedule(static)
			for (int i = 0; i < (int)numberOfPoints; i++)
			{
				glm::vec3 p = points[i];
				int vx = (int)((p.x - volumeMin.x) * invVoxelSize);
				int vy = (int)((p.y - volumeMin.y) * invVoxelSize);
				int vz = (int)((p.z - volumeMin.z) * invVoxelSize);

				if (vx >= 0 && vy >= 0 && vz >= 0)
				{
					threadVoxelKeys[tid].push_back(Morton3D::Encode(vx, vy, vz));
				}
			}
		}

		std::vector<uint64_t> uniqueVoxels;
		size_t totalSize = 0;
		for (auto& v : threadVoxelKeys) totalSize += v.size();
		uniqueVoxels.reserve(totalSize);
		for (auto& v : threadVoxelKeys) uniqueVoxels.insert(uniqueVoxels.end(), v.begin(), v.end());

		std::sort(uniqueVoxels.begin(), uniqueVoxels.end());
		auto last = std::unique(uniqueVoxels.begin(), uniqueVoxels.end());
		uniqueVoxels.erase(last, uniqueVoxels.end());
		TE(DedupVoxels);

		// ----------------------------------------------------------------
		// [Step 2] Create Blocks (Optimized & Parallelized)
		// ----------------------------------------------------------------
		TS(CreateBlocks);

		std::vector<std::vector<DataBlockKey>> threadBlockKeys(maxThreads);
		const int VpB_Shift = 3;

#pragma omp parallel
		{
			int tid = omp_get_thread_num();
			threadBlockKeys[tid].reserve(uniqueVoxels.size() / maxThreads / 2);

			int last_min_bx = -999, last_max_bx = -999;
			int last_min_by = -999, last_max_by = -999;
			int last_min_bz = -999, last_max_bz = -999;

#pragma omp for schedule(static)
			for (int i = 0; i < (int)uniqueVoxels.size(); i++)
			{
				uint64_t vKey = uniqueVoxels[i];
				uint32_t vx, vy, vz;
				Morton3D::Decode(vKey, vx, vy, vz);

				int min_bx = ((int)vx - iOffset) >> VpB_Shift;
				int max_bx = ((int)vx + iOffset) >> VpB_Shift;
				int min_by = ((int)vy - iOffset) >> VpB_Shift;
				int max_by = ((int)vy + iOffset) >> VpB_Shift;
				int min_bz = ((int)vz - iOffset) >> VpB_Shift;
				int max_bz = ((int)vz + iOffset) >> VpB_Shift;

				if (min_bx == last_min_bx && max_bx == last_max_bx &&
					min_by == last_min_by && max_by == last_max_by &&
					min_bz == last_min_bz && max_bz == last_max_bz)
				{
					continue;
				}

				for (int bz = min_bz; bz <= max_bz; bz++)
				{
					for (int by = min_by; by <= max_by; by++)
					{
						for (int bx = min_bx; bx <= max_bx; bx++)
						{
							if (bx >= 0 && by >= 0 && bz >= 0)
							{
								threadBlockKeys[tid].push_back(Morton3D::Encode(bx, by, bz));
							}
						}
					}
				}
				last_min_bx = min_bx; last_max_bx = max_bx;
				last_min_by = min_by; last_max_by = max_by;
				last_min_bz = min_bz; last_max_bz = max_bz;
			}
		}

		std::vector<DataBlockKey> neededBlocks;
		size_t totalBlocks = 0;
		for (auto& v : threadBlockKeys) totalBlocks += v.size();
		neededBlocks.reserve(totalBlocks);
		for (auto& v : threadBlockKeys) neededBlocks.insert(neededBlocks.end(), v.begin(), v.end());

		std::sort(neededBlocks.begin(), neededBlocks.end());
		auto lastBlockIt = std::unique(neededBlocks.begin(), neededBlocks.end());
		neededBlocks.erase(lastBlockIt, neededBlocks.end());

		dataBlocks.reserve(neededBlocks.size());
		for (auto key : neededBlocks) CreateDataBlock(key);

		TE(CreateBlocks);

		// ----------------------------------------------------------------
		// [Step 3] Fill Voxels with Data (Values + Colors + Normals + SDF)
		// ----------------------------------------------------------------
		TS(FillVoxels_Data);

		const int BlockSizeInt = 8;
		const float halfVoxel = voxelSize * 0.5f; // 복셀 중심 보정값

#pragma omp parallel
		{
			DataBlockKey lastKey = (DataBlockKey)-1;
			DataBlock* lastBlock = nullptr;

#pragma omp for schedule(dynamic, 512)
			for (int i = 0; i < (int)numberOfPoints; i++)
			{
				glm::vec3 p = points[i];
				glm::vec3 c = hasColors ? colors[i] : glm::vec3(0.0f);
				glm::vec3 n = hasNormals ? normals[i] : glm::vec3(0.0f);

				// Normal이 없으면 위쪽을 향한다고 가정하거나, Unsigned Distance 계산을 위해 0 벡터 처리
				bool validNormal = hasNormals && (glm::length2(n) > 1e-6f);

				int vx = (int)((p.x - volumeMin.x) * invVoxelSize);
				int vy = (int)((p.y - volumeMin.y) * invVoxelSize);
				int vz = (int)((p.z - volumeMin.z) * invVoxelSize);

				int start_x = vx - iOffset; int end_x = vx + iOffset;
				int start_y = vy - iOffset; int end_y = vy + iOffset;
				int start_z = vz - iOffset; int end_z = vz + iOffset;

				int min_bx = start_x >> 3; int max_bx = end_x >> 3;
				int min_by = start_y >> 3; int max_by = end_y >> 3;
				int min_bz = start_z >> 3; int max_bz = end_z >> 3;

				for (int bz = min_bz; bz <= max_bz; bz++)
				{
					for (int by = min_by; by <= max_by; by++)
					{
						for (int bx = min_bx; bx <= max_bx; bx++)
						{
							DataBlockKey currentKey = Morton3D::Encode(bx, by, bz);
							DataBlock* block = nullptr;

							if (currentKey == lastKey) { block = lastBlock; }
							else { block = GetBlockReadonly(currentKey); lastKey = currentKey; lastBlock = block; }

							if (!block) continue;

							// 블록의 월드 좌표 원점 (미리 계산)
							glm::vec3 blockOrigin = block->blockMin;

							int b_start_x = bx << 3;
							int b_start_y = by << 3;
							int b_start_z = bz << 3;

							int l_start_x = std::max(start_x, b_start_x) - b_start_x;
							int l_end_x = std::min(end_x, b_start_x + BlockSizeInt - 1) - b_start_x;
							int l_start_y = std::max(start_y, b_start_y) - b_start_y;
							int l_end_y = std::min(end_y, b_start_y + BlockSizeInt - 1) - b_start_y;
							int l_start_z = std::max(start_z, b_start_z) - b_start_z;
							int l_end_z = std::min(end_z, b_start_z + BlockSizeInt - 1) - b_start_z;

							while (block->lock.test_and_set(std::memory_order_acquire)) {}

							for (int lz = l_start_z; lz <= l_end_z; lz++)
							{
								// Z축 좌표 미리 계산 (최적화)
								float voxelWorldZ = blockOrigin.z + (float)lz * voxelSize + halfVoxel;
								int z_idx = lz << 6;

								for (int ly = l_start_y; ly <= l_end_y; ly++)
								{
									float voxelWorldY = blockOrigin.y + (float)ly * voxelSize + halfVoxel;
									int y_idx = ly << 3;

									for (int lx = l_start_x; lx <= l_end_x; lx++)
									{
										// 1. 복셀의 중심 월드 좌표 계산
										float voxelWorldX = blockOrigin.x + (float)lx * voxelSize + halfVoxel;
										glm::vec3 voxelCenter(voxelWorldX, voxelWorldY, voxelWorldZ);

										// 2. 벡터(Point -> Voxel) 계산
										glm::vec3 diff = voxelCenter - p;

										// 3. SDF 계산
										float dist = 0.0f;
										if (validNormal)
										{
											// Dot Product: 법선 방향으로의 투영 거리 (Signed)
											// 양수: 표면 밖, 음수: 표면 안
											dist = glm::dot(diff, n);
										}
										else
										{
											// 법선이 없으면 단순 유클리드 거리 (Unsigned)
											dist = glm::length(diff);
										}

										// 4. Update 호출 (dist 전달)
										block->UpdateVoxelSafe(z_idx | y_idx | lx, dist, c, n, hasColors, hasNormals);
									}
								}
							}
							block->lock.clear(std::memory_order_release);
						}
					}
				}
			}
		}
		TE(FillVoxels_Data);
	}

	/*void ShowBlocks()
	{
		for (auto& [key, block] : dataBlocks)
		{
			auto center = block.blockMin + glm::vec3(voxelSize * VpBHalf);
			VD::AddWiredBox("datablock", center, glm::vec3(voxelSize * VpB), Color::red());

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
								+ glm::vec3(0.5f * voxelSize);

							VD::AddWiredBox("datablock", vcenter, glm::vec3(voxelSize), Color::yellow());
						}
					}
				}
			}
		}
	}*/

	void ShowBlocks()
	{
		for (auto& [key, block] : dataBlocks)
		{
			auto center = block.blockMin + glm::vec3(voxelSize * VpBHalf);
			VD::AddWiredBox("datablock", center, glm::vec3(voxelSize * VpB), Color::red());

			for (size_t z = 0; z < VpB; z++)
			{
				for (size_t y = 0; y < VpB; y++)
				{
					for (size_t x = 0; x < VpB; x++)
					{
						auto voxel = block.voxels[z * VpB * VpB + y * VpB + x];
						if (voxel.valid)
						{
							if (voxelSize * 0.5f > fabsf(voxel.signedDistance))
							{
								auto vcenter = block.blockMin
									+ glm::vec3((float)x * voxelSize, (float)y * voxelSize, (float)z * voxelSize)
									+ glm::vec3(0.5f * voxelSize);

								// Display Voxel Color
								glm::vec3 vColor = (voxel.color.x == 0 && voxel.color.y == 0 && voxel.color.z == 0)
									? Color::yellow()
									: voxel.color;

								VD::AddSphere("voxel", vcenter, voxelSize * 0.5f, glm::vec4(vColor, 1.0f));
							}
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
			Feather.GetComponent<CameraManipulatorTrackball>(entity)->OnMouseButton(event);
			});
		Feather.GetRegistry().emplace<EventCallback<MouseWheelEvent>>(cam, cam, [](Entity entity, const MouseWheelEvent& event) {
			Feather.GetRegistry().get<CameraManipulatorTrackball>(entity).OnMouseWheel(event);
			});
	}
#pragma endregion

	Feather.AddOnInitializeCallback([&]() {
		{
			PLYFormat ply;
			ply.Deserialize("D:\\Debug\\PLY\\inputA.ply");
			auto [minx, miny, minz] = ply.GetAABBMin();

			// 시각화 (선택 사항)
			for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
			{
				auto x = ply.GetPoints()[i * 3 + 0];
				auto y = ply.GetPoints()[i * 3 + 1];
				auto z = ply.GetPoints()[i * 3 + 2];
				// VD::AddSphere("point", {x, y, z}, 0.02f, Color::white());
			}

			SparseDataBlock sdb;

			TS(Sorting);
			// [중요] 정렬을 통해 공간적 지역성 확보
			SortDataByMorton(
				ply.GetPoints(),
				ply.GetNormals(),
				ply.GetColors(),
				0.1f, // voxelSize
				8     // VpB
			);
			TE(Sorting);

			// 최적화된 FromPoints 호출
			sdb.FromPoints(
				(glm::vec3*)ply.GetPoints().data(),
				(glm::vec3*)ply.GetNormals().data(),
				(glm::vec3*)ply.GetColors().data(),
				ply.GetPoints().size() / 3,
				{ minx, miny, minz }
			);

			sdb.ShowBlocks();

			alog("Total DataBlocks : %s\n", FormatWithCommas(sdb.dataBlocks.size()).c_str());
			alog("Total Memory : %s bytes\n", FormatWithCommas(sdb.dataBlocks.size() * sizeof(DataBlock)).c_str());
		}
	});

	Feather.Run();
	Feather.Terminate();

	return 0;
}
