#include <iostream>
using namespace std;

#include <libFeather.h>

using VD = VisualDebugging;

int main(int argc, char** argv)
{
	cout << "AppFeather" << endl;

	Feather.Initialize(1920, 1080);

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
		{
			//auto appMain = Feather.CreateInstance<Entity>("AppMain");
			//auto appMainEventReceiver = Feather.CreateInstance<ComponentBase>();
			//appMainEventReceiver->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
			//	if (GLFW_KEY_ESCAPE == event.keyEvent.keyCode)
			//	{
			//		glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
			//	}
			//	});
		}

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

			Feather.GetRegistry().emplace<EventCallback<MousePositionEvent>>(gui, gui, [](Entity entity, const MousePositionEvent& event) {
				auto& component = Feather.GetRegistry().get<StatusPanel>(entity);
				component.mouseX = event.xpos;
				component.mouseY = event.ypos;
				});
		}
#pragma endregion

//#define RENDER_TRIANGLE
#ifdef RENDER_TRIANGLE
		{
			auto entity = Feather.CreateInstance<Entity>("Triangle");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs"));
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetShader(shader);

			renderable->AddVertex({ -1.0f, -1.0f, 0.0f });
			renderable->AddVertex({ 1.0f, -1.0f, 0.0f });
			renderable->AddVertex({ 0.0f, 1.0f, 0.0f });

			renderable->AddNormal({ 0.0f, 0.0f, 1.0f });
			renderable->AddNormal({ 0.0f, 0.0f, 1.0f });
			renderable->AddNormal({ 0.0f, 0.0f, 1.0f });

			renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
			renderable->AddColor({ 0.0f, 1.0f, 0.0f, 1.0f });
			renderable->AddColor({ 0.0f, 0.0f, 1.0f, 1.0f });

			renderable->AddIndex(0);
			renderable->AddIndex(1);
			renderable->AddIndex(2);
		}
#endif // RENDER_TRIANGLE

//#define RENDER_VOXELS_BOX
#ifdef RENDER_VOXELS_BOX
		{
			auto entity = Feather.CreateInstance<Entity>("Box");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetShader(shader);

			auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "one");
			renderable->AddIndices(indices);
			renderable->AddVertices(vertices);
			renderable->AddNormals(normals);
			renderable->AddColors(colors);
			renderable->AddUVs(uvs);

			int xCount = 100;
			int yCount = 100;
			int zCount = 100;
			int tCount = xCount * yCount * zCount;

			for (int i = 0; i < tCount; i++)
			{
				int z = i / (xCount * yCount);
				int y = (i % (xCount * yCount)) / xCount;
				int x = (i % (xCount * yCount)) % xCount;

				glm::mat4 model = glm::identity<glm::mat4>();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = glm::translate(model, glm::vec3(x, y, z));
				renderable->AddInstanceTransform(model);
			}

			renderable->EnableInstancing(tCount);
		}
#endif // RENDER_VOXELS_BOX

//#define RENDER_VOXELS_SPHERE
#ifdef RENDER_VOXELS_SPHERE
		{
			auto entity = Feather.CreateInstance<Entity>("Box");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetShader(shader);

			auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere("zero", 0.25f, 6, 6);
			//auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "half");
			renderable->AddIndices(indices);
			renderable->AddVertices(vertices);
			renderable->AddNormals(normals);
			renderable->AddColors(colors);
			renderable->AddUVs(uvs);

			int xCount = 100;
			int yCount = 100;
			int zCount = 100;
			int tCount = xCount * yCount * zCount;

			for (int i = 0; i < tCount; i++)
			{
				int z = i / (xCount * yCount);
				int y = (i % (xCount * yCount)) / xCount;
				int x = (i % (xCount * yCount)) % xCount;

				glm::mat4 model = glm::identity<glm::mat4>();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = glm::translate(model, glm::vec3(x, y, z));
				renderable->AddInstanceTransform(model);
			}

			renderable->EnableInstancing(tCount);

			renderable->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
				if (GLFW_KEY_M == event.keyEvent.keyCode)
				{
					auto renderable = dynamic_cast<Renderable*>(object);
					renderable->NextDrawingMode();
				}
				});
		}
#endif // RENDER_VOXELS_BOX

//#define LOAD_PLY
#ifdef LOAD_PLY
		{
			auto entity = Feather.CreateInstance<Entity>("Teeth");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs"));

			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Points);
			renderable->SetShader(shader);

			PLYFormat ply;
			ply.Deserialize("../../res/3D/Teeth.ply");
			ply.SwapAxisYZ();

			if(false == ply.GetPoints().empty())
				renderable->AddVertices((glm::vec3*)ply.GetPoints().data(), ply.GetPoints().size() / 3);
			if (false == ply.GetNormals().empty())
				renderable->AddNormals((glm::vec3*)ply.GetNormals().data(), ply.GetNormals().size() / 3);
			if (false == ply.GetColors().empty())
			{
				if (ply.UseAlpha())
				{
					renderable->AddColors((glm::vec4*)ply.GetColors().data(), ply.GetColors().size() / 4);
				}
				else
				{
					renderable->AddColors((glm::vec3*)ply.GetColors().data(), ply.GetColors().size() / 3);
				}
			}

			auto cameraManipulator = Feather.GetFirstInstance<CameraManipulatorTrackball>();
			auto camera = cameraManipulator->SetCamera();
			auto [x, y, z] = ply.GetAABBCenter();
			camera->SetEye({ x,y,z + cameraManipulator->GetRadius() });
			camera->SetTarget({ x,y,z });
		}
#endif // LOAD_PLY

		/*
		{
			PLYFormat ply;
			ply.Deserialize("../../res/3D/Teeth_Full.ply");
			ply.SwapAxisYZ();

			auto entity = Feather.CreateInstance<Entity>("Box");
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetShader(shader);

			auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere("zero", 0.05f, 6, 6);
			//auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "half");
			renderable->AddIndices(indices);
			renderable->AddVertices(vertices);
			renderable->AddNormals(normals);
			renderable->AddColors(colors);
			renderable->AddUVs(uvs);

			ui32 tCount = ply.GetPoints().size() / 3;

			for (int i = 0; i < tCount; i++)
			{
				auto x = ply.GetPoints()[i * 3];
				auto y = ply.GetPoints()[i * 3 + 1];
				auto z = ply.GetPoints()[i * 3 + 2];

				glm::mat4 model = glm::identity<glm::mat4>();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = glm::translate(model, glm::vec3(x, y, z));
				renderable->AddInstanceTransform(model);
			}

			renderable->EnableInstancing(tCount);

			renderable->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
				if (GLFW_KEY_M == event.keyEvent.keyCode)
				{
					auto renderable = dynamic_cast<Renderable*>(object);
					renderable->NextDrawingMode();
				}
				});
		}
		*/
		
		{
			struct Point
			{
				glm::vec3 position;
				glm::vec3 normal;
				glm::vec3 color;
			};
			ALPFormat<Point> alp;
			if (false == alp.Deserialize("../../res/3D/Teeth_Full.alp"))
			{
				PLYFormat ply;
				ply.Deserialize("../../res/3D/Teeth_Full.ply");
				ply.SwapAxisYZ();

				vector<Point> points;
				for (size_t i = 0; i < ply.GetPoints().size() / 3; i++)
				{
					auto px = ply.GetPoints()[i * 3];
					auto py = ply.GetPoints()[i * 3 + 1];
					auto pz = ply.GetPoints()[i * 3 + 2];

					auto nx = ply.GetNormals()[i * 3];
					auto ny = ply.GetNormals()[i * 3 + 1];
					auto nz = ply.GetNormals()[i * 3 + 2];

					auto cx = ply.GetColors()[i * 3];
					auto cy = ply.GetColors()[i * 3 + 1];
					auto cz = ply.GetColors()[i * 3 + 2];

					points.push_back({ {px, py, pz}, {nx, ny, nz}, {cx, cy, cz} });
				}

				alp.AddPoints(points);
				alp.Serialize("../../res/3D/Teeth_Full.alp");
			}

			{
				auto entity = Feather.GetRegistry().create();
				auto& renderable = Feather.GetRegistry().emplace<Renderable>(entity);
				renderable.Initialize(Renderable::GeometryMode::Triangles);
				{
					auto shader = Feather.CreateShader("Instancing", File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
					renderable.AddShader(shader);
				}
				{
					auto shader = Feather.CreateShader("InstancingWithoutNormal", File("../../res/Shaders/InstancingWithoutNormal.vs"), File("../../res/Shaders/InstancingWithoutNormal.fs"));
					renderable.AddShader(shader);
				}
				renderable.SetActiveShaderIndex(1);

				auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere({0.0f, 0.0f, 0.0f}, 0.05f, 6, 6);
				//auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox("zero", "half");
				renderable.AddIndices(indices);
				renderable.AddVertices(vertices);
				renderable.AddNormals(normals);
				renderable.AddColors(colors);
				renderable.AddUVs(uvs);

				for (auto& p : alp.GetPoints())
				{
					auto r = p.color.x;
					auto g = p.color.y;
					auto b = p.color.z;
					auto a = 1.f;

					renderable.AddInstanceColor(glm::vec4(r, g, b, a));
					renderable.AddInstanceNormal(p.normal);

					glm::mat4 model = glm::identity<glm::mat4>();
					model[0][0] = 1.5f;
					model[1][1] = 1.5f;
					model[2][2] = 1.5f;
					model = glm::translate(model, p.position);
					renderable.AddInstanceTransform(model);
				}

				renderable.EnableInstancing(alp.GetPoints().size());

				//renderable->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
				//	if (GLFW_KEY_M == event.keyEvent.keyCode)
				//	{
				//		auto renderable = dynamic_cast<Renderable*>(object);
				//		renderable->NextDrawingMode();
				//	}
				//	else if (GLFW_KEY_1 == event.keyEvent.keyCode)
				//	{
				//		auto renderable = dynamic_cast<Renderable*>(object);
				//		renderable->SetActiveShaderIndex(0);
				//	}
				//	else if (GLFW_KEY_2 == event.keyEvent.keyCode)
				//	{
				//		auto renderable = dynamic_cast<Renderable*>(object);
				//		renderable->SetActiveShaderIndex(1);
				//	}
				//	});
			}
		}

		VD::AddBox("TestBox", { 0.0f, 2.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 10.0f, 10.0f, 10.0f }, { 1.0f, 1.0f, 0.0f, 1.0f });
		});

	Feather.Run();

	Feather.Terminate();

	return 0;
}
