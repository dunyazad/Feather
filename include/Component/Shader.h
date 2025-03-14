#pragma once

#include <FeatherCommon.h>
#include <File.h>
#include <Component/ComponentBase.h>

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

    inline GLint GetUniformLocation(const string& name) { return glGetUniformLocation(shaderProgram, name.c_str()); }
    inline void UniformM4(GLint location, const MiniMath::M4& m) { glUniformMatrix4fv(location, 1, GL_FALSE, (float*)m.m); }

private:
    GLuint shaderProgram = UINT_MAX;
};
