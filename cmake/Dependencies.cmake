include(FetchContent)

if(WIN32)
    set(CMAKE_USE_LONG_PATHS ON CACHE BOOL "Enable long path support on Windows" FORCE)

    # 일부 CMake 버전(3.30 이하)에서 FetchContent rule 깨짐 방지
    if(POLICY CMP0169)
        cmake_policy(SET CMP0169 OLD)
    endif()
endif()

# RxTx
include(FetchContent)
FetchContent_Declare(
    RxTx
    GIT_REPOSITORY https://github.com/dunyazad/RxTx.git
    GIT_TAG main
)
FetchContent_MakeAvailable(RxTx)

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

if (NOT TARGET imgui_build)
    add_library(imgui_build STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )

    target_include_directories(imgui_build PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )

    target_link_libraries(imgui_build PUBLIC glfw glad)
endif()


# implot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(implot)

# imnodes
include(FetchContent)
FetchContent_Declare(
    imnodes
    GIT_REPOSITORY https://github.com/Nelarius/imnodes.git
    GIT_TAG master
)

# CMake 3.30에서 FetchContent_Populate() 경고 방지
# (이 정책 설정은 경고만 억제함 — 안전함)
if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)
endif()

FetchContent_GetProperties(imnodes)
if(NOT imnodes_POPULATED)
    FetchContent_Populate(imnodes)

    add_library(imnodes STATIC
        ${imnodes_SOURCE_DIR}/imnodes.cpp
    )

    # imnodes는 imgui를 include해야 하므로 둘 다 지정
    target_include_directories(imnodes PUBLIC
        ${imnodes_SOURCE_DIR}
        ${imgui_SOURCE_DIR}
    )

    target_link_libraries(imnodes PUBLIC imgui_build)
endif()



# entt
FetchContent_Declare(
    entt
    GIT_REPOSITORY https://github.com/skypjack/entt.git
    GIT_TAG        v3.15.0
)
FetchContent_MakeAvailable(entt)

# stb
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(stb)
