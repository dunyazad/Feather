#include <iostream>

#include <libFeather.h>

using VD = VisualDebugging;

using namespace libRxTx;

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
					memcpy(&flipped[y * width],
						&depth[(height - 1 - y) * width],
						width * sizeof(float));

				// 깊이값 최소/최대 구하기 (자동 정규화)
				float minD = 1.0f, maxD = 0.0f;
				for (float v : flipped) {
					if (v < minD) minD = v;
					if (v > maxD) maxD = v;
				}
				printf("[Depth] min=%.6f, max=%.6f, range=%.6f\n", minD, maxD, maxD - minD);
				float range = std::max(maxD - minD, 1e-6f);

				// RAW 저장 (optional)
				FILE* f = fopen("depth.raw", "wb");
				fwrite(flipped.data(), sizeof(float), width * height, f);
				fclose(f);
				printf("[DepthMap] Saved to depth.raw (min=%.5f, max=%.5f)\n", minD, maxD);

				// pseudo-color PNG
				std::vector<unsigned char> rgb(width * height * 3);
				for (int i = 0; i < width * height; ++i)
				{
					float norm = (flipped[i] - minD) / range;  // 정규화
					unsigned char r, g, b;
					DepthToColor(norm, r, g, b);
					rgb[i * 3 + 0] = r;
					rgb[i * 3 + 1] = g;
					rgb[i * 3 + 2] = b;
				}

				stbi_write_png("depth_color.png", width, height, 3, rgb.data(), width * 3);
				printf("[DepthMap] Saved to depth_color.png (normalized pseudo-color)\n");
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
					//auto ray = manipulator->GetCamera()->ScreenPointToRay(event.xpos, event.ypos, w->GetWidth(), w->GetHeight());
					//VD::Clear("PickingRay");
					//VD::AddLine("PickingRay", ray.origin, ray.direction * 500.0f, { 1.0f, 0.0f, 0.0f, 1.0f }, { 1.0f, 0.0f, 0.0f, 1.0f });

					int fbWidth, fbHeight;
					glfwGetFramebufferSize(w->GetGLFWwindow(), &fbWidth, &fbHeight);

					int readX = static_cast<int>(event.xpos);
					int readY = fbHeight - static_cast<int>(event.ypos) - 1;

					float depth = 0.0f;
					glReadPixels(readX, readY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
					printf("Depth at (%d, %d) [fb corrected: (%d, %d)] = %f\n",
						(int)event.xpos, (int)event.ypos, readX, readY, depth);

					GLint depthBits = 0;
					glGetIntegerv(GL_DEPTH_BITS, &depthBits);
					printf("Default framebuffer depth bits: %d\n", depthBits);


					//int width, height;
					//glfwGetFramebufferSize(w->GetGLFWwindow(), &width, &height);

					//// OpenGL 좌표계 보정 (y 뒤집기)
					//int readX = static_cast<int>(event.xpos);
					//int readY = height - static_cast<int>(event.ypos) - 1;

					//float depth = 0.0f;
					//glReadPixels(readX, readY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

					//printf("Depth at (%d, %d) [corrected: (%d, %d)] = %f\n",
					//	(int)event.xpos, (int)event.ypos, readX, readY, depth);
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

			for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
			{
				auto x = ply.GetPoints()[3 * i + 0];
				auto y = ply.GetPoints()[3 * i + 1];
				auto z = ply.GetPoints()[3 * i + 2];
				glm::vec3 position = glm::vec3(x, y, z);

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

				if (ply.UseAlpha())
				{
					auto r = ply.GetColors()[4 * i + 0];
					auto g = ply.GetColors()[4 * i + 1];
					auto b = ply.GetColors()[4 * i + 2];
					auto a = ply.GetColors()[4 * i + 3];

					renderable->AddInstanceColor({ r, g, b, a });
				}
				else
				{
					auto r = ply.GetColors()[3 * i + 0];
					auto g = ply.GetColors()[3 * i + 1];
					auto b = ply.GetColors()[3 * i + 2];
					renderable->AddInstanceColor({ r, g, b, 1.0f });
				}
				renderable->AddInstanceNormal(normal);

				glm::mat4 tm = glm::identity<glm::mat4>();
				glm::mat4 rot = glm::mat4(1.0f);
				if (glm::length(normal) > 0.0001f)
				{
					glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(0, 0, 1), normal));
					float angle = acos(glm::dot(glm::normalize(normal), glm::vec3(0, 0, 1)));
					if (glm::length(axis) > 0.0001f)
						rot = glm::rotate(glm::mat4(1.0f), angle, axis);
				}

				tm = glm::translate(tm, position) * rot * glm::scale(glm::mat4(1.0f), glm::vec3(0.1f));

				renderable->AddInstanceTransform(tm);
				renderable->IncreaseNumberOfInstances();
			}

			Feather.CreateEventCallback<KeyEvent>(entity, [](Entity entity, const KeyEvent& event) {
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

		{
			// UDP Receiver
			RxTx udp(Protocol::UDP);
			udp.Init();
			udp.SetMode(UdpMode::Broadcast);
			udp.Bind(5000); // 자동 NIC 선택
			udp.OnReceive([](const std::string& msg) {
				std::cout << "[UDP Received] " << msg << std::endl;
				});
			udp.Start();

			// UDP Sender
			RxTx sender(Protocol::UDP);
			sender.Init();
			sender.SetMode(UdpMode::Broadcast);
			sender.Bind(0); // 자동 포트 선택
			sender.Start();
			sender.SendToAll("Hello from SendToAll!");

			// TCP Server
			RxTx server(Protocol::TCP);
			server.Init();
			server.Bind(9000);
			server.Listen();
			server.OnConnect([](const std::string& ip) { std::cout << "[TCP] Connect: " << ip << std::endl; });
			server.OnReceive([](const std::string& msg) { std::cout << "[TCP] Received: " << msg << std::endl; });
			server.OnDisconnect([](const std::string& ip) { std::cout << "[TCP] Disconnect: " << ip << std::endl; });
			server.Start();

			// TCP Client
			RxTx client(Protocol::TCP);
			client.Init();
			client.Connect("127.0.0.1", 9000);
			client.Start();
			client.SendPacket("Hello Packet TCP!");

			std::this_thread::sleep_for(std::chrono::seconds(3));

			udp.Stop();
			sender.Stop();
			server.Stop();
			client.Stop();
		}
		});

	Feather.Run();

	Feather.Terminate();

	return 0;
}
