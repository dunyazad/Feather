#pragma once

#include <libFeatherCommon.h>

class Shader
{
public:
	Shader();
	~Shader();

    void Initialize();
    void Terminate();

    void CheckShaderCompileErrors(GLuint shader, const std::string& type);

    void Use();

private:
    GLuint shaderProgram = UINT_MAX;

    // Vertex Shader (��ȯ ��� ����)
    const char* vertexShaderSource = R"(
    // Vertex Shader (��ȯ ��� �� ī�޶� ����)
    #version 330 core

    layout (location = 0) in vec3 aPos;  // ���� ��ġ (Vertex Position)

    uniform mat4 transform;  // ȸ�� ��ȯ ���
    uniform mat4 projection; // ���� ���
    uniform mat4 view;       // ī�޶� �� ���

    void main() {
        // ��ȯ ����� ���� ��ġ ��ȯ
        gl_Position = projection * view * transform * vec4(aPos, 1.0);
    }
)";

    // Fragment Shader
    const char* fragmentShaderSource = R"(
    // Fragment Shader (���� ����)
    #version 330 core

    out vec4 FragColor;

    void main() {
        // �ܼ��� ���� ���� (������)
        FragColor = vec4(0.8, 0.3, 0.2, 1.0); // RGB �� (������)
    }
)";
};
