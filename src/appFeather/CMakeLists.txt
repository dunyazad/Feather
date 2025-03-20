cmake_minimum_required(VERSION 3.20)
project(appFeather)

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

add_executable(appFeather
    src/main.cpp
)

target_include_directories(appFeather PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/libFeather/include
)

target_link_libraries(appFeather
    libFeather
)

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
