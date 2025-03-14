#include <iostream>
using namespace std;

#include <libFeather.h>

int main(int argc, char** argv)
{
	cout << "AppFeather" << endl;

	Feather.Initialize(1920, 1080);

	GLuint VAO, VBO;

	auto w = Feather.GetFeatherWindow();

	Feather.AddOnInitializeCallback([&]() {
		{
			auto appMain = Feather.CreateInstance<Entity>("AppMain");
			auto appMainEventReceiver = Feather.CreateInstance<ComponentBase>();
			//appMain->AddComponent(appMainEventReceiver);
			appMainEventReceiver->AddEventHandler(EventType::KeyPress, [&](const Event& event) {
				if (GLFW_KEY_ESCAPE == event.parameters.key.keyCode)
				{
					glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
				}
				});
		}
		{
			auto camera = Feather.CreateInstance<Entity>("Camera");
			auto perspectiveCamera = Feather.CreateInstance<PerspectiveCamera>();
			//camera->AddComponent(perspectiveCamera);

			auto cameraManipulator = Feather.CreateInstance<CameraManipulatorOrbit>();
			//camera->AddComponent(cameraManipulator);
			cameraManipulator->SetCamera(perspectiveCamera);
		}

		{
			auto gui = Feather.CreateInstance<Entity>("GUI");
			auto statusPanel = Feather.CreateInstance<StatusPanel>();
			//gui->AddComponent(statusPanel);
		}

		//{
		//	auto entity = Feather.CreateInstance<Entity>("Triangle");
		//	auto shader = Feather.CreateInstance<Shader>();
		//	shader->Initialize(File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs"));
		//	auto renderable = Feather.CreateInstance<Renderable>();
		//	renderable->Initialize(Renderable::GeometryMode::Triangles);
		//	renderable->SetShader(shader);
		//	//entity->AddComponent(shader);
		//	//entity->AddComponent(renderable);

		//	renderable->AddVertex({ -1.0f, -1.0f, 0.0f });
		//	renderable->AddVertex({ 1.0f, -1.0f, 0.0f });
		//	renderable->AddVertex({ 0.0f, 1.0f, 0.0f });

		//	renderable->AddNormal({ 0.0f, 0.0f, 1.0f });
		//	renderable->AddNormal({ 0.0f, 0.0f, 1.0f });
		//	renderable->AddNormal({ 0.0f, 0.0f, 1.0f });

		//	renderable->AddColor({ 1.0f, 0.0f, 0.0f, 1.0f });
		//	renderable->AddColor({ 0.0f, 1.0f, 0.0f, 1.0f });
		//	renderable->AddColor({ 0.0f, 0.0f, 1.0f, 1.0f });

		//	renderable->AddIndex(0);
		//	renderable->AddIndex(1);
		//	renderable->AddIndex(2);
		//}

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

			for (int i = 0; i < 10; i++)
			{
				MiniMath::M4 model = MiniMath::M4::identity();
				model = MiniMath::translate(model, MiniMath::V3(i * 2.0f, 0.0f, 0.0f));
				renderable->AddInstanceTransform(model);
			}

			renderable->EnableInstancing(10);
		}

		});

	Feather.Run();

	Feather.Terminate();

	return 0;
}
