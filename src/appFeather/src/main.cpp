#include <iostream>
using namespace std;

#include <libFeather.h>

int main(int argc, char** argv)
{
	cout << "AppFeather" << endl;

	Feather::GetInstance().Initialize(1920, 1080);

    GLuint VAO, VBO;

    auto w = Feather::GetInstance().GetFeatherWindow();

    Feather::GetInstance().GetSystem<InputSystem>()->AddKeyReleaseEventHandler(
    GLFW_KEY_ESCAPE,
    [](GLFWwindow* glfwWindow, KeyEvent keyEvent) {
        glfwSetWindowShouldClose(Feather::GetInstance().GetFeatherWindow()->GetGLFWwindow(), true);
    });

    Feather::GetInstance().GetSystem<InputSystem>()->AddKeyPressEventHandler(
    GLFW_KEY_LEFT,
    [](GLFWwindow* glfwWindow, KeyEvent keyEvent) {
            auto entity = Feather::GetInstance().GetEntity(0);
            auto perspectiveCamera = entity->GetComponent<PerspectiveCamera>(0);
            perspectiveCamera->GetEye().x -= 1.0f;
    });
    Feather::GetInstance().GetSystem<InputSystem>()->AddKeyPressEventHandler(
        GLFW_KEY_RIGHT,
        [](GLFWwindow* glfwWindow, KeyEvent keyEvent) {
            auto entity = Feather::GetInstance().GetEntity(0);
            auto perspectiveCamera = entity->GetComponent<PerspectiveCamera>(0);
            perspectiveCamera->GetEye().x += 1.0f;
        });

    Feather::GetInstance().AddOnInitializeCallback([&]() {
        auto camera = Feather::GetInstance().CreateEntity("Camera");
        auto perspectiveCamera = Feather::GetInstance().CreatePerspectiveCamera();

        camera->AddComponentID(perspectiveCamera);

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
