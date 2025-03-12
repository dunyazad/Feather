#include <Core/FeatherWindow.h>

FeatherWindow::FeatherWindow()
{
}

FeatherWindow::~FeatherWindow()
{
}

GLFWwindow* FeatherWindow::Initialize(ui32 width, ui32 height)
{
    this->width = width;
    this->height = height;

    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return nullptr;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE); // Allows glBegin/glEnd


    window = glfwCreateWindow(width, height, "Feather", nullptr, nullptr);

    if (!window)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD\n";
        return nullptr;
    }

    return window;
}

void FeatherWindow::Terminate()
{
    glfwDestroyWindow(window);
    glfwTerminate();
}

void FeatherWindow::Frame()
{

}
