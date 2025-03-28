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

#define i8_max (INT8_MAX)
#define i8_min (-INT8_MAX)
#define i16_max (INT16_MAX)
#define i16_min (-INT16_MAX)
#define i32_max (INT32_MAX)
#define i32_min (-INT32_MAX)
#define i64_max (INT64_MAX)
#define i64_min (-INT64_MAX)

#define ui8_max (UINT8_MAX)
#define ui16_max (UINT16_MAX)
#define ui32_max (UINT32_MAX)
#define ui64_max (UINT64_MAX)

#define f32_max (FLT_MAX)
#define f32_min (-FLT_MAX)
#define f64_max (DBL_MAX)
#define f64_min (-DBL_MAX)

struct f2
{
	f32 x;
	f32 y;
};

struct f3
{
	f32 x;
	f32 y;
	f32 z;
};

struct f4
{
	f32 x;
	f32 y;
	f32 z;
	f32 w;
};

struct d2
{
	f64 x;
	f64 y;
};

struct d3
{
	f64 x;
	f64 y;
	f64 z;
};

struct d4
{
	f64 x;
	f64 y;
	f64 z;
	f64 w;
};

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

struct PointP
{
	f3 position;
};

struct PointPN
{
	f3 position;
	f3 normal;
};

struct PointPNC
{
	f3 position;
	f3 normal;
	f3 color;
};
