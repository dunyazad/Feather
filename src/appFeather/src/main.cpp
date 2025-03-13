#include <iostream>
using namespace std;

#include <libFeather.h>

int main(int argc, char** argv)
{
	cout << "AppFeather" << endl;

	Feather::GetInstance().Initialize(1920, 1080);

	GLuint VAO, VBO;

	auto w = Feather::GetInstance().GetFeatherWindow();

	Feather::GetInstance().AddOnInitializeCallback([&]() {
		auto appMain = Feather::GetInstance().CreateEntity("AppMain");
		auto appMainEventReceiver = Feather::GetInstance().CreateComponent<ComponentBase>();
		appMain->AddComponent(appMainEventReceiver);
		appMainEventReceiver->AddEventHandler(EventType::KeyPress, [&](const Event& event) {
			if (GLFW_KEY_ESCAPE == event.parameters.key.keyCode)
			{
				glfwSetWindowShouldClose(Feather::GetInstance().GetFeatherWindow()->GetGLFWwindow(), true);
			}
			});

		auto camera = Feather::GetInstance().CreateEntity("Camera");
		auto perspectiveCamera = Feather::GetInstance().CreateComponent<PerspectiveCamera>();
		camera->AddComponent(perspectiveCamera);

		auto cameraManipulator = Feather::GetInstance().CreateComponent<CameraManipulatorOrbit>();
		camera->AddComponent(cameraManipulator);
		cameraManipulator->SetCamera(perspectiveCamera);

		auto statusPanel = Feather::GetInstance().CreateComponent<StatusPanel>();
		appMain->AddComponent(statusPanel);

		//auto cameraEventReceiver = Feather::GetInstance().CreateComponent<EventReceiver>();
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

	Feather::GetInstance().Run();

	Feather::GetInstance().Terminate();

	return 0;
}
