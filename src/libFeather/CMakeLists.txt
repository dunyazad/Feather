cmake_minimum_required(VERSION 3.20)
project(libFeather)

include(FetchContent)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MSVC-specific options
if (MSVC)
    add_compile_options(/wd4819)
    add_compile_options(/wd4005)
    add_compile_options(/std:c++17 /Zc:__cplusplus)
endif()

# Fetch dependencies
macro(fetch_dependency name repo tag)
    FetchContent_Declare(
        ${name}
        GIT_REPOSITORY ${repo}
        GIT_TAG ${tag}
    )
    FetchContent_GetProperties(${name})
    if(NOT ${name}_POPULATED)
        FetchContent_MakeAvailable(${name})
    endif()
endmacro()

# Disable unwanted builds for dependencies
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "Disable GLFW examples" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "Disable GLFW tests" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "Disable GLFW documentation" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "Disable GLFW installation target" FORCE)

set(IMPLOT_BUILD_EXAMPLES OFF CACHE BOOL "Disable ImPlot examples" FORCE)

set(LIBUV_BUILD_TESTS OFF CACHE BOOL "Disable libuv tests" FORCE)

fetch_dependency(glad "https://github.com/Dav1dde/glad.git" "v0.1.36")
fetch_dependency(glfw "https://github.com/glfw/glfw.git" "3.3.8")
fetch_dependency(imgui "https://github.com/ocornut/imgui.git" "docking")
fetch_dependency(implot "https://github.com/epezent/implot.git" "v0.15")
fetch_dependency(entt "https://github.com/skypjack/entt.git" "v3.11.1")
fetch_dependency(glm "https://github.com/g-truc/glm.git" "0.9.9.8")

set(BUILD_TESTING OFF CACHE BOOL "Disable Eigen testing" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "Disable Eigen testing" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Disable Eigen documentation" FORCE)
set(EIGEN_BUILD_EXAMPLES OFF CACHE BOOL "Disable Eigen examples" FORCE)
set(EIGEN_LEAVE_TEST_IN_ALL_TARGET OFF CACHE BOOL "Disable Eigen tests" FORCE)
fetch_dependency(eigen "https://gitlab.com/libeigen/eigen.git" "3.4.0")

fetch_dependency(libuv "https://github.com/libuv/libuv.git" "v1.46.0")

set(source_list
    ${CMAKE_SOURCE_DIR}/include/libFeather.h
    ${CMAKE_SOURCE_DIR}/include/FeatherCommon.h
    ${CMAKE_SOURCE_DIR}/include/Feather.h

    ${CMAKE_SOURCE_DIR}/include/Event.h
    ${CMAKE_SOURCE_DIR}/include/EventDispatcher.h
    ${CMAKE_SOURCE_DIR}/include/EventHandler.h
    ${CMAKE_SOURCE_DIR}/include/FeatherWindow.h
    ${CMAKE_SOURCE_DIR}/include/File.h
    ${CMAKE_SOURCE_DIR}/include/GeometryBuilder.h
    ${CMAKE_SOURCE_DIR}/include/MiniMath.h
    ${CMAKE_SOURCE_DIR}/include/Monitor.h
    ${CMAKE_SOURCE_DIR}/include/Serialization.hpp
    ${CMAKE_SOURCE_DIR}/include/TypeDefinition.h

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
    src/MiniMath.cpp
    src/EventDispatcher.cpp
    src/EventHandler.cpp
    src/FeatherWindow.cpp
    src/File.cpp
    src/GeometryBuilder.cpp
    src/Monitor.cpp
    
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
    ${glfw_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${implot_SOURCE_DIR}
    ${entt_SOURCE_DIR}/single_include
    ${glm_SOURCE_DIR}
    ${eigen_SOURCE_DIR}
    ${libuv_SOURCE_DIR}/include
)

# Enable Docking feature for ImGui
target_compile_definitions(libFeather PUBLIC IMGUI_ENABLE_DOCKING IMGUI_DEFINE_MATH_OPERATORS)

# Link dependencies
target_link_libraries(libFeather
    glfw
    glad
    uv
    #libuv
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
set_target_properties(uninstall PROPERTIES FOLDER "CMakePredefinedTargets")
set_target_properties(glad PROPERTIES FOLDER "Libraries")
set_target_properties(glad-generate-files PROPERTIES FOLDER "Libraries")
set_target_properties(glfw PROPERTIES FOLDER "Libraries")
set_target_properties(uv PROPERTIES FOLDER "Libraries")
set_target_properties(uv_a PROPERTIES FOLDER "Libraries")
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
