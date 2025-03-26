cmake_minimum_required(VERSION 3.20)
project(Feather)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/external)
set(Feather_SOURCE_DIR ${CMAKE_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_SOURCE_DIR}/bin/Debug)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_SOURCE_DIR}/bin/Release)

# Set module path for custom cmake files
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Include dependencies
include(Dependencies)

add_subdirectory(src/libFeather)
add_subdirectory(src/appFeather)
add_subdirectory(src/cuappFeather)

if(MSVC)
    # /bigobj 옵션 추가 (컴파일러 오류 방지)
    target_compile_options(libFeather PRIVATE /bigobj)
    target_compile_options(appFeather PRIVATE /bigobj)
    target_compile_options(cuappFeather PRIVATE /bigobj)
endif()
