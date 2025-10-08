#include <iostream>
//using namespace std;

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

#pragma region Joystick
		{
			auto entity = Feather.CreateEntity("Joystick");
			auto component = Feather.CreateComponent<Joystick>(entity, Feather.GetHWND());
		}
#pragma endregion

#pragma region Ground Plane
		{
			auto entity = Feather.CreateEntity("Ground Plane");
			auto component = Feather.CreateComponent<Renderable>(entity);
			component->Initialize(Renderable::GeometryMode::Triangles);
			component->SetDrawingMode(Renderable::DrawingMode::WireFrameOverSolid);

			auto [indices, vertices, normals, colors, uvs] =
				GeometryBuilder::BuildPlane(1000, 1000, 100, 100, { 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, Color::white());
			component->AddIndices(indices.data(), indices.size());
			component->AddVertices(vertices.data(), vertices.size());
			component->AddNormals(normals.data(), normals.size());
			component->AddColors(colors.data(), colors.size());
			component->AddUVs(uvs.data(), uvs.size());

			component->AddShader(Feather.CreateShader("Default", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
			component->AddShader(Feather.CreateShader("Flat", File("../../res/Shaders/Flat.vs"), File("../../res/Shaders/Flat.fs")));
			component->SetActiveShaderIndex(0);
		}
#pragma endregion

#pragma region Cube 0
		{
			auto entity = Feather.CreateEntity("Cube 0");
			auto renderable = Feather.CreateComponent<Renderable>(entity);
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetDrawingMode(Renderable::DrawingMode::WireFrameOverSolid);

			auto [indices, vertices, normals, colors, uvs] =
				GeometryBuilder::BuildBox({ 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f }, Color::green());
			renderable->AddIndices(indices.data(), indices.size());
			renderable->AddVertices(vertices.data(), vertices.size());
			renderable->AddNormals(normals.data(), normals.size());
			renderable->AddColors(colors.data(), colors.size());
			renderable->AddUVs(uvs.data(), uvs.size());

			renderable->AddShader(Feather.CreateShader("Default", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
			renderable->AddShader(Feather.CreateShader("Flat", File("../../res/Shaders/Flat.vs"), File("../../res/Shaders/Flat.fs")));
			renderable->SetActiveShaderIndex(0);

			auto transform = Feather.CreateComponent<Transform>(entity);
			transform->SetLocalTransformMatrix(glm::translate(glm::identity<glm::mat4>(), { 0.0f, 5.0f, 0.0f }));

			Feather.CreateEventCallback<KeyEvent>(entity, [](Entity entity, const KeyEvent& event) {
				auto cube = Feather.GetEntityByName("Cube 0");
				auto transform = Feather.GetComponent<Transform>(cube); 
				if (GLFW_KEY_U == event.keyCode)
				{
					transform->SetLocalTransformMatrix(glm::translate(transform->GetLocalTransformMatrix(), { 0.0f, 0.0f, -0.1f }));
				}
				else if (GLFW_KEY_O == event.keyCode)
				{
					transform->SetLocalTransformMatrix(glm::translate(transform->GetLocalTransformMatrix(), { 0.0f, 0.0f, 0.1f }));
				}
				else if (GLFW_KEY_K == event.keyCode)
				{
					transform->SetLocalTransformMatrix(glm::translate(transform->GetLocalTransformMatrix(), { 0.0f, -0.1f, 0.0f }));
				}
				else if (GLFW_KEY_I == event.keyCode)
				{
					transform->SetLocalTransformMatrix(glm::translate(transform->GetLocalTransformMatrix(), { 0.0f, 0.1f, 0.0f }));
				}
				else if (GLFW_KEY_J == event.keyCode)
				{
					transform->SetLocalTransformMatrix(glm::translate(transform->GetLocalTransformMatrix(), { -0.1f, 0.0f, 0.0f }));
				}
				else if (GLFW_KEY_L == event.keyCode)
				{
					transform->SetLocalTransformMatrix(glm::translate(transform->GetLocalTransformMatrix(), { 0.1f, 0.0f, 0.0f }));
				}
				});


			Feather.CreateEventCallback<JoystickEvent>(entity, [](Entity entity, const JoystickEvent& event) {
				printf("Joystick: AxisX=%f AxisY=%f AxisZ=%f RotX=%f RotY=%f RotZ=%f\n",
					event.AxisX, event.AxisY, event.AxisZ, event.RotX, event.RotY, event.RotZ);
				});
		}
#pragma endregion

#pragma region Cube 1
		{
			auto entity = Feather.CreateEntity("Cube 1");
			auto renderable = Feather.CreateComponent<Renderable>(entity);
			renderable->Initialize(Renderable::GeometryMode::Triangles);
			renderable->SetDrawingMode(Renderable::DrawingMode::WireFrameOverSolid);

			auto [indices, vertices, normals, colors, uvs] =
				GeometryBuilder::BuildBox({ 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f }, Color::blue());
			renderable->AddIndices(indices.data(), indices.size());
			renderable->AddVertices(vertices.data(), vertices.size());
			renderable->AddNormals(normals.data(), normals.size());
			renderable->AddColors(colors.data(), colors.size());
			renderable->AddUVs(uvs.data(), uvs.size());

			renderable->AddShader(Feather.CreateShader("Default", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
			renderable->AddShader(Feather.CreateShader("Flat", File("../../res/Shaders/Flat.vs"), File("../../res/Shaders/Flat.fs")));
			renderable->SetActiveShaderIndex(0);

			auto transform = Feather.CreateComponent<Transform>(entity);
			transform->SetLocalTransformMatrix(glm::translate(glm::identity<glm::mat4>(), { 0.0f, 0.0f, 5.0f }));
		}
#pragma endregion

		{
			auto cube0 = Feather.GetEntityByName("Cube 0");
			auto transform0 = Feather.GetComponent<Transform>(cube0);
			auto cube1 = Feather.GetEntityByName("Cube 1");
			auto transform1 = Feather.GetComponent<Transform>(cube1);
			transform1->SetParent(transform0);
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

//#pragma region Status Panel
//		{
//			auto gui = Feather.GetRegistry().create();
//			auto statusPanel = Feather.GetRegistry().emplace<StatusPanel>(gui);
//
//			Feather.GetRegistry().emplace<EventCallback<MousePositionEvent>>(gui, gui, [](Entity entity, const MousePositionEvent& event) {
//				auto& component = Feather.GetRegistry().get<StatusPanel>(entity);
//				component.mouseX = event.xpos;
//				component.mouseY = event.ypos;
//				});
//		}
//#pragma endregion
//
//#pragma region Control Panel
//		{
//			auto entity = Feather.CreateEntity("Control Panel");
//			auto controlPanel = Feather.CreateComponent<ControlPanel>(entity, "Control Panel");
//		}
//#pragma endregion
		});

	Feather.Run();

	Feather.Terminate();

	return 0;
}
