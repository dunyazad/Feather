#pragma once

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

#define f32_max (FLT_MAX)
#define f32_min (-FLT_MAX)

struct FrameBufferResizeEvent
{
	i32 width;
	i32 height;
};

struct KeyEvent
{
	i32 keyCode;
	i32 scanCode;
	i32 action;
	i32 mods;
};

struct MousePositionEvent
{
	f64 xpos;
	f64 ypos;
};

struct MouseButtonEvent
{
	i32 button;
	i32 action;
	i32 mods;
	f64 xpos;
	f64 ypos;
};

struct MouseWheelEvent
{
	f64 xoffset;
	f64 yoffset;
};
