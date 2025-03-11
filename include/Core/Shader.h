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

    // Vertex Shader (변환 행렬 적용)
    const char* vertexShaderSource = R"(
    // Vertex Shader (변환 행렬 및 카메라 설정)
    #version 330 core

    layout (location = 0) in vec3 aPos;  // 입점 위치 (Vertex Position)

    uniform mat4 transform;  // 회전 변환 행렬
    uniform mat4 projection; // 투영 행렬
    uniform mat4 view;       // 카메라 뷰 행렬

    void main() {
        // 변환 행렬을 통해 위치 변환
        gl_Position = projection * view * transform * vec4(aPos, 1.0);
    }
)";

    // Fragment Shader
    const char* fragmentShaderSource = R"(
    // Fragment Shader (색상 설정)
    #version 330 core

    out vec4 FragColor;

    void main() {
        // 단순한 색상 설정 (붉은색)
        FragColor = vec4(0.8, 0.3, 0.2, 1.0); // RGB 값 (붉은색)
    }
)";
};
