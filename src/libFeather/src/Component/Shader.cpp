#include <Component/Shader.h>

Shader::Shader(ComponentID id)
    : ComponentBase(id)
{
}

Shader::~Shader()
{
}

void Shader::Initialize(const File& vsFile, const File& fsFile)
{
    if (0 == vsFile.GetFileLength() || 0 == fsFile.GetFileLength()) return;

    string vs(vsFile.GetFileLength() + 1, 0);
    vsFile.Read(vs.data(), vsFile.GetFileLength());

    string fs(fsFile.GetFileLength() + 1, 0);
    fsFile.Read(fs.data(), fsFile.GetFileLength());

    Initialize(vs, fs);
}

void Shader::Initialize(const string& vs, const string& fs)
{
    if (vs.empty() || fs.empty()) return;

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    const GLchar* vsSource = vs.c_str();
    glShaderSource(vertexShader, 1, &vsSource, nullptr);
    glCompileShader(vertexShader);
    CheckShaderCompileErrors(vertexShader, "VERTEX");

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    const GLchar* fsSource = fs.c_str();
    glShaderSource(fragmentShader, 1, &fsSource, nullptr);
    glCompileShader(fragmentShader);
    CheckShaderCompileErrors(fragmentShader, "FRAGMENT");

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    CheckShaderCompileErrors(shaderProgram, "PROGRAM");

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

void Shader::Terminate()
{
    if (UINT_MAX != shaderProgram)
    {
        glDeleteProgram(shaderProgram);
        shaderProgram = UINT_MAX;
    }
}

void Shader::CheckShaderCompileErrors(GLuint shader, const std::string& type)
{
    GLint success;
    GLchar infoLog[1024];
    if (type == "VERTEX" || type == "FRAGMENT") {
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (!success) {
            glGetShaderInfoLog(shader, 1024, NULL, infoLog);
            std::cerr << type << " Shader compilation failed: " << infoLog << std::endl;
        }
    }
    else if (type == "PROGRAM") {
        glGetProgramiv(shader, GL_LINK_STATUS, &success);
        if (!success) {
            glGetProgramInfoLog(shader, 1024, NULL, infoLog);
            std::cerr << "Shader Program linking failed: " << infoLog << std::endl;
        }
    }
}

void Shader::Use()
{
    glUseProgram(shaderProgram);
}
