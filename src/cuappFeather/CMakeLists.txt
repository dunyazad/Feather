cmake_minimum_required(VERSION 3.5)

# Project name
project(cuappFeather LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler=/wd4819")

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

find_package(CUDAToolkit REQUIRED)

enable_language(CUDA)

set(CMAKE_CUDA_ARCHITECTURES 75 86) # Replace with architectures suitable for your GPU(s)

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


set(cuda_source_list
    src/main.cuh
    src/main.cu
)

set(source_list
    src/main.cpp
)

set_source_files_properties(${cuda_source_list} PROPERTIES LANGUAGE CUDA)

add_library(cuda_objects OBJECT ${cuda_source_list})

add_executable(cuappFeather
 ${source_list}
$<TARGET_OBJECTS:cuda_objects>  # Link CUDA object files here
)

target_include_directories(cuappFeather PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libFeather/include
    ${glad_SOURCE_DIR}/include
    ${glad_BINARY_DIR}/include
    ${glfw_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${implot_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    # "External"
    # "External/eigen"
    # "External/nvapi"
    # #"External/nanoflann"
    # #"External/nanovdb"
    # ${VTK_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
)

target_include_directories(cuda_objects PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libFeather/include
    ${glad_SOURCE_DIR}/include
    ${glad_BINARY_DIR}/include
    ${glfw_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${implot_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    # "External"
    # "External/eigen"
    # "External/nvapi"
    # #"External/nanoflann"
    # #"External/nanovdb"
    # ${VTK_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_link_directories(cuappFeather PRIVATE ${Feather_SOURCE_DIR}/bin/Debug)
else()
    target_link_directories(cuappFeather PRIVATE ${Feather_SOURCE_DIR}/bin/Release)
endif()

target_link_libraries(cuappFeather
    #${VTK_LIBRARIES}
    #${HDIAPI_LIBRARIES}
    libFeather
    CUDA::cudart CUDA::cuda_driver
    CUDA::cusolver CUDA::cusparse
    CUDA::cufft
    #${CMAKE_CURRENT_SOURCE_DIR}/External/nvapi/amd64/nvapi64.lib
)

target_link_libraries(cuda_objects PRIVATE
    #${VTK_LIBRARIES}
    libFeather
    CUDA::cufft
)


target_compile_options(cuda_objects PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:--extended-lambda>)

if(MSVC)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT cuappFeather)
    target_compile_options(cuappFeather PRIVATE /bigobj)
endif()

set_target_properties(cuappFeather PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON  # Change to ON if needed
    CMAKE_CUDA_ARCHITECTURES "60;70;80"
    CUDA_STANDARD 11  # Adjust based on your CUDA version
)

# vtk_module_autoinit(
#     TARGETS cuappFeather
#     MODULES ${VTK_LIBRARIES}
# )

# Group source files in IDE
function(assign_source_group)
    foreach(_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_source}")
        else()
            set(_source_rel "${_source}")
        endif()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("${_source_path_msvc}" FILES "${_source}")
    endforeach()
endfunction(assign_source_group)

assign_source_group(${cuda_source_list})
assign_source_group(${source_list})



set_property(TARGET appFeather PROPERTY VS_DEPLOYMENT_TARGETS libFeather)

# 실행 파일의 기본 출력 디렉토리 설정
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${Feather_SOURCE_DIR}/bin)

# 각 빌드 타입별로 별도 디렉토리 설정
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${Feather_SOURCE_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${Feather_SOURCE_DIR}/bin/Release)

# Visual Studio에서 작업 디렉토리 설정
set_target_properties(${PROJECT_NAME} PROPERTIES
    VS_DEBUGGER_WORKING_DIRECTORY "$<IF:$<CONFIG:Debug>,${CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG},${CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE}>"
)
