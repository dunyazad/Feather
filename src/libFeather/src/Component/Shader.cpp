#include <Component/Shader.h>

Shader::Shader()
{
}

Shader::~Shader()
{
    Terminate();
}

void Shader::Initialize(const File& vsFile, const File& fsFile)
{
    if (0 == vsFile.GetFileLength() || 0 == fsFile.GetFileLength()) return;

    std::string vs(vsFile.GetFileLength() + 1, 0);
    vsFile.Read(vs.data(), vsFile.GetFileLength());

    std::string fs(fsFile.GetFileLength() + 1, 0);
    fsFile.Read(fs.data(), fsFile.GetFileLength());

    Initialize(vs, "", fs);
}

void Shader::Initialize(const File& vsFile, const File& gsFile, const File& fsFile)
{
    if (0 == vsFile.GetFileLength() || 0 == fsFile.GetFileLength()) return;

    std::string vs(vsFile.GetFileLength() + 1, 0);
    vsFile.Read(vs.data(), vsFile.GetFileLength());

    std::string gs(gsFile.GetFileLength() + 1, 0);
    gsFile.Read(gs.data(), gsFile.GetFileLength());

    std::string fs(fsFile.GetFileLength() + 1, 0);
    fsFile.Read(fs.data(), fsFile.GetFileLength());

    Initialize(vs, gs, fs);
}

void Shader::Initialize(const std::string& vs, const std::string& gs, const std::string& fs)
{
    if (vs.empty() || fs.empty()) return;

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    const GLchar* vsSource = vs.c_str();
    glShaderSource(vertexShader, 1, &vsSource, nullptr);
    glCompileShader(vertexShader);
    CheckShaderCompileErrors(vertexShader, "VERTEX");

    GLuint geometryShader = 0;
    if (false == gs.empty())
    {
        geometryShader = glCreateShader(GL_GEOMETRY_SHADER);
        const GLchar* gsSource = gs.c_str();
        glShaderSource(geometryShader, 1, &gsSource, nullptr);
        glCompileShader(geometryShader);
        CheckShaderCompileErrors(geometryShader, "GEOMETRY");
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    const GLchar* fsSource = fs.c_str();
    glShaderSource(fragmentShader, 1, &fsSource, nullptr);
    glCompileShader(fragmentShader);
    CheckShaderCompileErrors(fragmentShader, "FRAGMENT");

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    if (false == gs.empty())
    {
        glAttachShader(shaderProgram, geometryShader);
    }
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    CheckShaderCompileErrors(shaderProgram, "PROGRAM");

    glDeleteShader(vertexShader);
    if (false == gs.empty())
    {
        glDeleteShader(geometryShader);
    }
    glDeleteShader(fragmentShader);
}

void Shader::Terminate()
{
    if (ui32_max != shaderProgram) // 헤더의 초기값과 맞춤 (UINT_MAX or ui32_max)
    {
        glDeleteProgram(shaderProgram);
        shaderProgram = ui32_max;
    }
}

void Shader::CheckShaderCompileErrors(GLuint shader, const std::string& type)
{
    GLint success;
    GLchar infoLog[1024];

    if (type == "PROGRAM")
    {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, NULL, infoLog);
            std::cerr << "Shader Program linking failed: " << infoLog << std::endl;
        }
    }
    else // VERTEX, GEOMETRY, FRAGMENT 등 쉐이더 컴파일 오류
    {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            std::cerr << type << " Shader compilation failed: " << infoLog << std::endl;
        }
    }
}

void Shader::Use()
{
    if (shaderProgram != ui32_max)
        glUseProgram(shaderProgram);
}