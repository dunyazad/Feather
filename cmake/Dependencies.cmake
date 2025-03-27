include(FetchContent)

# glad
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG        v0.1.36
)
FetchContent_MakeAvailable(glad)

# glfw
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Do not build GLFW examples" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "Do not build GLFW tests" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "Do not build GLFW documentation" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "Disable GLFW installation target" FORCE)
set(GLFW_BUILD_SHARED_LIBS OFF CACHE BOOL "Build GLFW as a static library" FORCE)

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
)
FetchContent_MakeAvailable(glfw)

# imgui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
)
FetchContent_MakeAvailable(imgui)

# implot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.15
)
FetchContent_MakeAvailable(implot)

# entt
FetchContent_Declare(
    entt
    GIT_REPOSITORY https://github.com/skypjack/entt.git
    GIT_TAG        v3.11.1
)
FetchContent_MakeAvailable(entt)
