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
			auto appMain = Feather.CreateEntity("AppMain");
			auto appMainEventReceiver = Feather.CreateComponent<ComponentBase>();
			appMain->AddComponent(appMainEventReceiver);
			appMainEventReceiver->AddEventHandler(EventType::KeyPress, [&](const Event& event) {
				if (GLFW_KEY_ESCAPE == event.parameters.key.keyCode)
				{
					glfwSetWindowShouldClose(Feather.GetFeatherWindow()->GetGLFWwindow(), true);
				}
				});
		}
		{
			auto camera = Feather.CreateEntity("Camera");
			auto perspectiveCamera = Feather.CreateComponent<PerspectiveCamera>();
			camera->AddComponent(perspectiveCamera);

			auto cameraManipulator = Feather.CreateComponent<CameraManipulatorOrbit>();
			camera->AddComponent(cameraManipulator);
			cameraManipulator->SetCamera(perspectiveCamera);
		}

		{
			auto gui = Feather.CreateEntity("GUI");
			auto statusPanel = Feather.CreateComponent<StatusPanel>();
			gui->AddComponent(statusPanel);
		}

		{
			auto mesh = Feather.CreateEntity("mesh");
			auto shader = Feather.CreateComponent<Shader>();
			shader->Initialize(File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs"));
		}
		
		//auto cameraEventReceiver = Feather.CreateComponent<EventReceiver>();
		//camera->AddComponent(cameraEventReceiver);
		//cameraEventReceiver->AddEventHandler(EventType::KeyPress, [&](const Event& event) {});


		/*
		glEnable(GL_DEPTH_TEST);
		glViewport(0, 0, w->GetWidth(), w->GetHeight());

		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &VBO);
		glBindVertexArray(VAO);

		float vertices[] = {
			-0.5f, -0.5f,  0.0f,
			 0.5f, -0.5f,  0.0f,
			 0.0f,  0.5f,  0.0f
		};

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
		*/
		});

	Feather.Run();

	Feather.Terminate();

	return 0;
}
