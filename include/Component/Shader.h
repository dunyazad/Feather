#pragma once

#include <FeatherCommon.h>
#include <File.h>

class Shader
{
public:
	Shader();
	virtual ~Shader();

    void Initialize(const File& vsFile, const File& fsFile);
    void Initialize(const File& vsFile, const File& gsFile, const File& fsFile);
    void Initialize(const std::string& vs, const std::string& gs, const std::string& fs);
    void Terminate();

    void CheckShaderCompileErrors(GLuint shader, const std::string& type);

    void Use();

    inline GLint GetUniformLocation(const std::string& name) { return glGetUniformLocation(shaderProgram, name.c_str()); }
    inline void UniformInt(GLint location, int i) { glUniform1i(location, i); }
    inline void UniformV3(GLint location, const glm::vec3& v) { glUniform3fv(location, 1, (float*)&v); }
    inline void UniformM4(GLint location, const glm::mat4& m) { glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(m)); }

private:
    GLuint shaderProgram = ui32_max;

    std::string vertexShaderFileName;
    std::string geometryShaderFileName;
    std::string fragmentShaderFileName;
};
