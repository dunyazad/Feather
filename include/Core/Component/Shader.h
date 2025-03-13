#pragma once

#include <FeatherCommon.h>
#include <Core/File.h>
#include <Core/Component/ComponentBase.h>

class Shader : public ComponentBase
{
public:
	Shader(ComponentID id);
	virtual ~Shader();

    void Initialize(const File& vsFile, const File& fsFile);
    void Initialize(const string& vs, const string& fs);
    void Terminate();

    void CheckShaderCompileErrors(GLuint shader, const std::string& type);

    void Use();

private:
    GLuint shaderProgram = UINT_MAX;

//    // Vertex Shader (변환 행렬 적용)
//    const char* vertexShaderSource = R"(
//    #version 330 core
//
//    layout (location = 0) in vec3 aPos;
//
//    uniform mat4 transform;
//    uniform mat4 projection;
//    uniform mat4 view;
//
//    void main() {
//        gl_Position = projection * view * transform * vec4(aPos, 1.0);
//    }
//)";
//
//    // Fragment Shader
//    const char* fragmentShaderSource = R"(
//    #version 330 core
//
//    out vec4 FragColor;
//
//    void main() {
//        FragColor = vec4(0.8, 0.3, 0.2, 1.0);
//    }
//)";
};
