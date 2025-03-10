#pragma once

#define _USE_MATH_DEFINES  // M_PI Ȱ��ȭ
#include <cmath>  // ���� ���� �Լ� �� ��� ����
#include <cstring>  // memset
#include <functional>
#include <iostream>
#include <vector>

using namespace std;

#include <glad/glad.h>
#include <GLFW/glfw3.h>

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
