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
#include <fstream>
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
#include <typeinfo>
#include <typeindex>
#include <unordered_map>
#include <unordered_set>
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

#define Feather libFeather::GetStaticInstance()

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

#include <MiniMath.h>

namespace Time
{
    chrono::steady_clock::time_point Now();

    uint64_t Microseconds(chrono::steady_clock::time_point& from, chrono::steady_clock::time_point& now);

    chrono::steady_clock::time_point End(chrono::steady_clock::time_point& from, const string& message = "", int number = -1);

    string DateTime();
}

#define alog(...) printf("\033[38;5;1m\033[48;5;15m(^(OO)^) /V/\033[0m\t" __VA_ARGS__)
#define alogt(tag, ...) printf("\033[38;5;1m\033[48;5;15m [%d] (^(OO)^) /V/\033[0m\t" tag, __VA_ARGS__)






enum class EventType
{
	None,
	FrameBufferResize,
	KeyPress,
	KeyRelease,
	MousePosition,
	MouseButtonPress,
	MouseButtonRelease,
	MouseWheel,
	NumberOfEventTypes
};

struct FrameBufferResizeParameters
{
	i32 width;
	i32 height;
};

struct KeyEventParameters
{
	i32 keyCode;
	i32 scanCode;
	i32 mods;
};

struct MousePositionEventParameters
{
	f64 xpos;
	f64 ypos;
};

struct MouseButtonEventParameters
{
	i32 button;
	i32 mods;
	f64 xpos;
	f64 ypos;
};

struct MouseWheelEventParameters
{
	f64 xoffset;
	f64 yoffset;
};

struct Event
{
	EventType type;
	union Parameters
	{
		KeyEventParameters key;
		MousePositionEventParameters mousePosition;
		MouseButtonEventParameters mouseButton;
		FrameBufferResizeParameters frameBufferResize;
		MouseWheelEventParameters mouseWheel;
	} parameters;
};

class FeatherObject
{
public:
	virtual ~FeatherObject() = default;

	static unordered_map<type_index, unordered_set<type_index>>& GetSubclassMap();
	static void RegisterClass(type_index baseType, type_index derivedType);
	static unordered_set<type_index> GetAllSubclasses(type_index baseType);

	virtual void OnEvent(const Event& event);
	virtual void SubscribeEvent(EventType eventType);
	virtual void AddEventHandler(EventType eventType, function<void(const Event&)> handler);

	inline const string& GetName() const { return name; }
	inline void SetName(const string& name) { this->name = name; }

protected:
	string name = "";
	map<EventType, vector<function<void(const Event&)>>> eventHandlers;

private:
	static unordered_map<type_index, unordered_set<type_index>> subclass_map;
};

template <typename Derived, typename Base>
class RegisterDerivation : public Base
{
public:
	template <typename... Args>
	RegisterDerivation(Args&&... args) : Base(std::forward<Args>(args)...)
	{
		static bool registered = []() {
			FeatherObject::RegisterClass(typeid(Base), typeid(Derived));
			return true;
		}();
	}
};
