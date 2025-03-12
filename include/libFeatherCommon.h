#pragma once

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
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
using namespace std;

#include <omp.h>

#define WIN32_LEAN_AND_MEAN
#define NO_MINMAX
#define NO_BYTE
#include <windows.h>
#include <shellapi.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32  // Win32 관련 기능을 활성화
#include <GLFW/glfw3native.h>

#include "imgui.h"
#include "imgui_internal.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include "implot.h"

typedef char i8;
typedef short i16;
typedef int i32;
typedef long i64;

typedef unsigned char ui8;
typedef unsigned short ui16;
typedef unsigned int ui32;
typedef unsigned long ui64;

typedef float f32;
typedef double f64;

typedef ui32 EntityID;
typedef ui32 ComponentID;

#include <Core/MiniMath.h>

namespace Time
{
    chrono::steady_clock::time_point Now();

    uint64_t Microseconds(chrono::steady_clock::time_point& from, chrono::steady_clock::time_point& now);

    chrono::steady_clock::time_point End(chrono::steady_clock::time_point& from, const string& message = "", int number = -1);

    string DateTime();
}

#define alog(...) printf("\033[38;5;1m\033[48;5;15m(^(OO)^) /V/\033[0m\t" __VA_ARGS__)
#define alogt(tag, ...) printf("\033[38;5;1m\033[48;5;15m [%d] (^(OO)^) /V/\033[0m\t" tag, __VA_ARGS__)

class libFeather;
class FeatherWindow;

class Entity;

class GUISystem;
class InputSystem;
class RenderSystem;
class ImmediateModeRenderSystem;
class SystemBase;

class Feather
{
public:
    static FeatherWindow* GetFeatherWindow();
    static GLFWwindow* GetGLFWWindow();

    static Entity* CreateEntity();
    static Entity* GetEntity(EntityID id);

    static GUISystem* GetGUISystem();
    static InputSystem* GetInputSystem();
    static RenderSystem* GetRenderSystem();
    static ImmediateModeRenderSystem* GetImmediateModeRenderSystem();
    static SystemBase* GetSystem(const string& systemName);

private:
    Feather();
    ~Feather();
    static libFeather* s_instance;

public:
    friend class libFeather;
};