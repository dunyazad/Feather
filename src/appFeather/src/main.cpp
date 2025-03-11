#include <iostream>
using namespace std;

#include <libFeather.h>

int main(int argc, char** argv)
{
	cout << "AppFeather" << endl;

	libFeather libFeather;
	libFeather.Initialize(1920, 1080);

    GLuint VAO, VBO;

    auto w = libFeather.GetFeatherWindow();

    libFeather.AddOnInitializeCallback([&]() {
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
	});

	libFeather.Run();

	libFeather.Terminate();

	return 0;
}
