#include <iostream>
using namespace std;

#include <libFeather.h>

int main(int argc, char** argv)
{
	cout << "AppFeather" << endl;

	Feather.Initialize(1920, 1080);

	GLuint VAO, VBO;

	auto w = Feather.GetFeatherWindow();

	/*
	Feather.AddOnInitializeCallback([&]() {
		{
			auto appMainEventReceiver = Feather.CreateInstance<ComponentBase>();
			appMainEventReceiver->AddEventHandler(EventType::KeyPress, [&](const Event& event, FeatherObject* object) {
				if (GLFW_KEY_ESCAPE == event.keyEvent.keyCode)
				{
					glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
				}
				});
		}
		{
			auto perspectiveCamera = Feather.CreateInstance<PerspectiveCamera>();

			//auto cameraManipulator = Feather.CreateInstance<CameraManipulatorOrbit>();
			//cameraManipulator->SetCamera(perspectiveCamera);

			auto cameraManipulator = Feather.CreateInstance<CameraManipulatorTrackball>();
			cameraManipulator->SetCamera(perspectiveCamera);
		}

		{
			auto statusPanel = Feather.CreateInstance<StatusPanel>();
		}

//#define RENDER_TRIANGLE
#ifdef RENDER_TRIANGLE
		{
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

				MiniMath::M4 model = MiniMath::M4::identity();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = MiniMath::translate(model, MiniMath::V3(x, y, z));
				renderable->AddInstanceTransform(model);
			}

			renderable->EnableInstancing(tCount);
		}
#endif // RENDER_VOXELS_BOX

//#define RENDER_VOXELS_SPHERE
#ifdef RENDER_VOXELS_SPHERE
		{
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

				MiniMath::M4 model = MiniMath::M4::identity();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = MiniMath::translate(model, MiniMath::V3(x, y, z));
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
			auto shader = Feather.CreateInstance<Shader>();
			shader->Initialize(File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs"));

			auto renderable = Feather.CreateInstance<Renderable>();
			renderable->Initialize(Renderable::GeometryMode::Points);
			renderable->SetShader(shader);

			PLYFormat ply;
			ply.Deserialize("../../res/3D/Teeth.ply");
			ply.SwapAxisYZ();

			if(false == ply.GetPoints().empty())
				renderable->AddVertices((MiniMath::V3*)ply.GetPoints().data(), ply.GetPoints().size() / 3);
			if (false == ply.GetNormals().empty())
				renderable->AddNormals((MiniMath::V3*)ply.GetNormals().data(), ply.GetNormals().size() / 3);
			if (false == ply.GetColors().empty())
			{
				if (ply.UseAlpha())
				{
					renderable->AddColors((MiniMath::V4*)ply.GetColors().data(), ply.GetColors().size() / 4);
				}
				else
				{
					renderable->AddColors((MiniMath::V3*)ply.GetColors().data(), ply.GetColors().size() / 3);
				}
			}

			auto cameraManipulator = Feather.GetFirstInstance<CameraManipulatorTrackball>();
			auto camera = cameraManipulator->SetCamera();
			auto [x, y, z] = ply.GetAABBCenter();
			camera->SetEye({ x,y,z + cameraManipulator->GetRadius() });
			camera->SetTarget({ x,y,z });
		}
#endif // LOAD_PLY

		{
			PLYFormat ply;
			ply.Deserialize("../../res/3D/Teeth.ply");
			ply.SwapAxisYZ();

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

				MiniMath::M4 model = MiniMath::M4::identity();
				model.m[0][0] = 0.5f;
				model.m[1][1] = 0.5f;
				model.m[2][2] = 0.5f;
				model = MiniMath::translate(model, MiniMath::V3(x, y, z));
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

		});
	*/

	auto appMain = Feather.CreateEntity();
	EventDispatcher::RegisterEntityEvent<KeyEvent>(appMain, [w](KeyEvent keyEvent) {
		if (GLFW_KEY_ESCAPE == keyEvent.keyCode)
		{
			glfwSetWindowShouldClose(w->GetGLFWwindow(), true);
		}});

	auto gui = Feather.CreateEntity();
	auto statusPanel = Feather.CreateComponent<StatusPanel>(gui);

	Feather.Run();

	Feather.Terminate();

	return 0;
}
