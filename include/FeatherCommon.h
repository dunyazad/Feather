#pragma once

#ifndef NOMINMAX
#define NOMINMAX
#endif

//#define _HAS_STD_BYTE 0

#include <cstdio>
#include <stdio.h>
#include <limits.h>

#include <algorithm>
#include <chrono>
#include <filesystem>

#define _USE_MATH_DEFINES
#include <cmath>

#include <bitset>
#include <cstddef>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <typeinfo>
#include <typeindex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Color.hpp>

#include <omp.h>

#include <entt/entt.hpp>
#define _HAS_STD_BYTE 0

using Entity = entt::entity;
#define InvalidEntity ((Entity)ui32_max)

using Registry = entt::registry;
using Dispatcher = entt::dispatcher;

const std::string EmptyString = "";

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifndef NO_BYTE
#define NO_BYTE
#endif

#include <windows.h>
#include <shellapi.h>

#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>
#pragma comment(lib, "dinput8.lib")
#pragma comment(lib, "dxguid.lib")

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32  // Win32 관련 기능을 활성화
#include <GLFW/glfw3native.h>

#include "imgui.h"
#include "imgui_internal.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "implot.h"
#include "imnodes.h"

#define Feather libFeather::GetStaticInstance()

#include <TypeDefinitions.h>

#include <glm_include.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif
#define DEG2RAD (PI/180)
#define RAD2DEG (180/PI)

#ifndef XYZ
#define XYZ(v) (v).x, (v).y, (v).z
#endif
#ifndef XYZW
#define XYZW(v) (v).x, (v).y, (v).z, (v).w
#endif

namespace Time
{
    std::chrono::steady_clock::time_point Now();

    uint64_t Microseconds(std::chrono::steady_clock::time_point& from, std::chrono::steady_clock::time_point& now);

    std::chrono::steady_clock::time_point End(std::chrono::steady_clock::time_point& from, const std::string& message = "", int number = -1);

    std::string DateTime();
}

std::string Miliseconds(const std::chrono::steady_clock::time_point beginTime, const char* tag);

#define TS(name) auto time_##name = std::chrono::high_resolution_clock::now();
#define TE(name) std::cout << Miliseconds(time_##name, #name) << std::endl;

#define alog(...) printf("\033[38;5;1m\033[48;5;15m(^(OO)^) /V/\033[0m\t" __VA_ARGS__)
#define alogt(tag, ...) printf("\033[38;5;1m\033[48;5;15m [%d] (^(OO)^) /V/\033[0m\t" tag, __VA_ARGS__)
