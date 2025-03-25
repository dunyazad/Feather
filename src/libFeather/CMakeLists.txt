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

    ${CMAKE_SOURCE_DIR}/include/Entity.h
    ${CMAKE_SOURCE_DIR}/include/FeatherWindow.h
    ${CMAKE_SOURCE_DIR}/include/File.h
    ${CMAKE_SOURCE_DIR}/include/MiniMath.h
    ${CMAKE_SOURCE_DIR}/include/Monitor.h
    ${CMAKE_SOURCE_DIR}/include/Serialization.hpp
    
    ${CMAKE_SOURCE_DIR}/include/AsyncLoader.h
    ${CMAKE_SOURCE_DIR}/include/GeometryBuilder.h

    ${CMAKE_SOURCE_DIR}/include/Component/Components.h
    ${CMAKE_SOURCE_DIR}/include/Component/ComponentBase.h
    ${CMAKE_SOURCE_DIR}/include/Component/Camera.h
    ${CMAKE_SOURCE_DIR}/include/Component/CameraManipulator.h
    ${CMAKE_SOURCE_DIR}/include/Component/Renderable.h
    ${CMAKE_SOURCE_DIR}/include/Component/Shader.h
    ${CMAKE_SOURCE_DIR}/include/Component/Transform.h

    ${CMAKE_SOURCE_DIR}/include/Component/GUIComponent/GUIComponents.h
    ${CMAKE_SOURCE_DIR}/include/Component/GUIComponent/GUIComponentBase.h
    ${CMAKE_SOURCE_DIR}/include/Component/GUIComponent/StatusPanel.h

    ${CMAKE_SOURCE_DIR}/include/System/Systems.h
    ${CMAKE_SOURCE_DIR}/include/System/EventSystem.h
    ${CMAKE_SOURCE_DIR}/include/System/GUISystem.h
    ${CMAKE_SOURCE_DIR}/include/System/InputSystem.h
    ${CMAKE_SOURCE_DIR}/include/System/ImmediateModeRenderSystem.h
    ${CMAKE_SOURCE_DIR}/include/System/RenderSystem.h
    ${CMAKE_SOURCE_DIR}/include/System/SystemBase.h

    src/Feather.cpp
    src/FeatherCommon.cpp
    src/Entity.cpp
    src/MiniMath.cpp
    src/FeatherWindow.cpp
    src/File.cpp
    src/Monitor.cpp

    src/AsyncLoader.cpp
    src/GeometryBuilder.cpp
    
    src/Component/ComponentBase.cpp
    src/Component/Camera.cpp
    src/Component/CameraManipulator.cpp
    src/Component/Renderable.cpp
    src/Component/Shader.cpp
    src/Component/Transform.cpp

    src/Component/GUIComponent/GUIComponentBase.cpp
    src/Component/GUIComponent/StatusPanel.cpp

    src/System/EventSystem.cpp
    src/System/GUISystem.cpp
    src/System/InputSystem.cpp
    src/System/ImmediateModeRenderSystem.cpp
    src/System/RenderSystem.cpp
    src/System/SystemBase.cpp

    ${CMAKE_SOURCE_DIR}/res/Shaders/Default.vs
    ${CMAKE_SOURCE_DIR}/res/Shaders/Default.fs
    ${CMAKE_SOURCE_DIR}/res/Shaders/Instancing.vs
    ${CMAKE_SOURCE_DIR}/res/Shaders/Instancing.fs
    ${CMAKE_SOURCE_DIR}/res/Shaders/InstancingWithoutNormal.vs
    ${CMAKE_SOURCE_DIR}/res/Shaders/InstancingWithoutNormal.fs
    ${CMAKE_SOURCE_DIR}/res/Shaders/Line.vs
    ${CMAKE_SOURCE_DIR}/res/Shaders/Line.fs
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
    ${glad_BINARY_DIR}/include
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


set_target_properties(${PROJECT_NAME} PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG   ${Feather_SOURCE_DIR}/bin/Debug
    LIBRARY_OUTPUT_DIRECTORY_DEBUG   ${Feather_SOURCE_DIR}/bin/Debug
    RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${Feather_SOURCE_DIR}/bin/Debug

    ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${Feather_SOURCE_DIR}/bin/Release
    LIBRARY_OUTPUT_DIRECTORY_RELEASE ${Feather_SOURCE_DIR}/bin/Release
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${Feather_SOURCE_DIR}/bin/Release

    VS_DEBUGGER_WORKING_DIRECTORY "$<IF:$<CONFIG:Debug>,${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG},${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>"
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
