#include <iostream>
using namespace std;

#include <libFeather.h>

int main(int argc, char** argv)
{
	cout << "AppFeather" << endl;

	libFeather libFeather;
	libFeather.Initialize();

	auto w = libFeather.CreateWindow(1920, 1080);
    auto s = libFeather.CreateShader();

    GLuint VAO, VBO;

	w->AddOnInitializeCallback([&]() {
        glEnable(GL_DEPTH_TEST);  // 깊이 테스트 활성화
        glViewport(0, 0, w->GetWidth(), w->GetHeight());  // Viewport 설정

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);

        // 삼각형(Teapot 대용)
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

		});

	libFeather.Run();

	libFeather.Terminate();

	return 0;
}
