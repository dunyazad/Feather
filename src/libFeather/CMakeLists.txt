cmake_minimum_required(VERSION 3.20)
project(libFeather)

include(FetchContent)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MSVC-specific options
if (MSVC)
    add_compile_options(/std:c++17 /Zc:__cplusplus)
endif()

# Fetch dependencies
FetchContent_Declare(
    glad
    GIT_REPOSITORY https://github.com/Dav1dde/glad.git
    GIT_TAG        v0.1.36
)
FetchContent_MakeAvailable(glad)

set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Do not build GLFW examples" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "Do not build GLFW tests" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "Do not build GLFW documentation" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "Disable GLFW installation target" FORCE)
set(GLFW_BUILD_SHARED_LIBS OFF CACHE BOOL "Build GLFW as a static library" FORCE)  # Ensure static build

FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
)
FetchContent_MakeAvailable(imgui)

FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        v0.15
)
FetchContent_MakeAvailable(implot)

set(source_list
    ${CMAKE_SOURCE_DIR}/include/libFeather.h
    ${CMAKE_SOURCE_DIR}/include/FeatherCommon.h
    ${CMAKE_SOURCE_DIR}/include/Feather.h
    ${CMAKE_SOURCE_DIR}/include/Core/Entity.h
    ${CMAKE_SOURCE_DIR}/include/Core/MiniMath.h
    ${CMAKE_SOURCE_DIR}/include/Core/FeatherWindow.h
    ${CMAKE_SOURCE_DIR}/include/Core/File.h

    ${CMAKE_SOURCE_DIR}/include/Core/Component/Components.h
    ${CMAKE_SOURCE_DIR}/include/Core/Component/ComponentBase.h
    ${CMAKE_SOURCE_DIR}/include/Core/Component/Camera.h
    ${CMAKE_SOURCE_DIR}/include/Core/Component/CameraManipulator.h
    ${CMAKE_SOURCE_DIR}/include/Core/Component/Renderable.h
    ${CMAKE_SOURCE_DIR}/include/Core/Component/Shader.h
    ${CMAKE_SOURCE_DIR}/include/Core/Component/Transform.h

    ${CMAKE_SOURCE_DIR}/include/Core/Component/GUIComponent/GUIComponents.h
    ${CMAKE_SOURCE_DIR}/include/Core/Component/GUIComponent/GUIComponentBase.h
    ${CMAKE_SOURCE_DIR}/include/Core/Component/GUIComponent/StatusPanel.h

    ${CMAKE_SOURCE_DIR}/include/Core/System/Systems.h
    ${CMAKE_SOURCE_DIR}/include/Core/System/EventSystem.h
    ${CMAKE_SOURCE_DIR}/include/Core/System/GUISystem.h
    ${CMAKE_SOURCE_DIR}/include/Core/System/InputSystem.h
    ${CMAKE_SOURCE_DIR}/include/Core/System/ImmediateModeRenderSystem.h
    ${CMAKE_SOURCE_DIR}/include/Core/System/RenderSystem.h
    ${CMAKE_SOURCE_DIR}/include/Core/System/SystemBase.h

    src/Feather.cpp
    src/FeatherCommon.cpp
    src/Core/Entity.cpp
    src/Core/MiniMath.cpp
    src/Core/FeatherWindow.cpp
    src/Core/File.cpp
    
    src/Core/Component/ComponentBase.cpp
    src/Core/Component/Camera.cpp
    src/Core/Component/CameraManipulator.cpp
    src/Core/Component/Renderable.cpp
    src/Core/Component/Shader.cpp
    src/Core/Component/Transform.cpp

    src/Core/Component/GUIComponent/GUIComponentBase.cpp
    src/Core/Component/GUIComponent/StatusPanel.cpp

    src/Core/System/EventSystem.cpp
    src/Core/System/GUISystem.cpp
    src/Core/System/InputSystem.cpp
    src/Core/System/ImmediateModeRenderSystem.cpp
    src/Core/System/RenderSystem.cpp
    src/Core/System/SystemBase.cpp
)

# Define the library
add_library(libFeather
    ${source_list}

    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

source_group("Source Files\\imgui\\imgui" FILES
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
)

source_group("Source Files\\imgui\\imgui\\Backends" FILES
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

source_group("Source Files\\imgui\\implot" FILES
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)

# Include directories
target_include_directories(libFeather PUBLIC
    ${CMAKE_SOURCE_DIR}/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${glad_SOURCE_DIR}/include
    ${glfw_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${implot_SOURCE_DIR}
)

# Enable Docking feature for ImGui
target_compile_definitions(libFeather PUBLIC IMGUI_ENABLE_DOCKING IMGUI_DEFINE_MATH_OPERATORS)

# Link dependencies
target_link_libraries(libFeather
    glfw
    glad
)

# Group source files in IDE
function(assign_source_group)
    foreach(_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_SOURCE_DIR}" "${_source}")
        else()
            set(_source_rel "${_source}")
        endif()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("${_source_path_msvc}" FILES "${_source}")
    endforeach()
endfunction(assign_source_group)

assign_source_group(${source_list})

# Organizing the Visual Studio Solution Explorer
set_target_properties(glad PROPERTIES FOLDER "Libraries")
set_target_properties(glad-generate-files PROPERTIES FOLDER "Libraries")
set_target_properties(glfw PROPERTIES FOLDER "Libraries")
# Move update_mappings into Libraries folder (if it exists)
if (TARGET update_mappings)
    set_target_properties(update_mappings PROPERTIES FOLDER "Libraries")
endif()
#set_target_properties(glfw-static PROPERTIES FOLDER "Libraries")

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "$<IF:$<CONFIG:Debug>,${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG},${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>"
)
