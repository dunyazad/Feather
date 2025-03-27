#include <FeatherWindow.h>
#include <Feather.h>
#include <System/Systems.h>

FeatherWindow* FeatherWindow::s_instance = nullptr;

FeatherWindow::FeatherWindow()
{
    s_instance = this;
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

    glfwSetFramebufferSizeCallback(window, FrameBufferSizeCallback);
    
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

void FeatherWindow::FrameBufferSizeCallback(GLFWwindow* window, i32 width, i32 height)
{
    s_instance->width = width;
    s_instance->height = height;

    auto eventSystems = Feather.GetInstances<EventSystem>();
    if (eventSystems.empty()) return;

    auto eventSystem = *(eventSystems.begin());
    /*Event event;
    event.type = EventType::FrameBufferResize;
    event.frameBufferResizeEvent.width = width;
    event.frameBufferResizeEvent.height = height;

    eventSystem->DispatchEvent(event);*/
}
