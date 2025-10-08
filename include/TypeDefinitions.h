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

#ifndef PRIMITIVE_MAX
#define PRIMITIVE_MAX
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
#endif

struct f2
{
	f32 x = 0.0f;
	f32 y = 0.0f;
};

struct f3
{
	f32 x = 0.0f;
	f32 y = 0.0f;
	f32 z = 0.0f;
};

struct f4
{
	f32 x = 0.0f;
	f32 y = 0.0f;
	f32 z = 0.0f;
	f32 w = 0.0f;
};

struct d2
{
	f64 x = 0.0;
	f64 y = 0.0;
};

struct d3
{
	f64 x = 0.0;
	f64 y = 0.0;
	f64 z = 0.0;
};

struct d4
{
	f64 x = 0.0;
	f64 y = 0.0;
	f64 z = 0.0;
	f64 w = 0.0;
};

struct FrameEvent
{
	ui32 frameNo;
	f32 timeDelta;
};

struct FrameBufferResizeEvent
{
	i32 width = 0;
	i32 height = 0;
};

struct KeyEvent
{
	i32 keyCode = 0;
	i32 scanCode = 0;
	i32 action = 0;
	i32 mods = 0;
};

struct MousePositionEvent
{
	f64 xpos = 0.0;
	f64 ypos = 0.0;
};

struct MouseButtonEvent
{
	i32 button = 0;
	i32 action = 0;
	i32 mods = 0;
	f64 xpos = 0.0;
	f64 ypos = 0.0;
};

struct MouseWheelEvent
{
	f64 xoffset = 0.0;
	f64 yoffset = 0.0;
};

struct JoystickEvent
{
	float AxisX = 0.0f;
	float AxisY = 0.0f;
	float AxisZ = 0.0f;
	float RotX = 0.0f;
	float RotY = 0.0f;
	float RotZ = 0.0f;
	bool Buttons[16] = {
		false, false, false, false,
		false, false, false, false,
		false, false, false, false,
		false, false, false, false };
};

struct PointP
{
	f3 position = { 0.0f, 0.0f, 0.0f };
};

struct PointPN
{
	f3 position = { 0.0f, 0.0f, 0.0f };
	f3 normal = { 0.0f, 0.0f, 0.0f };
};

struct PointPNC
{
	f3 position = { 0.0f, 0.0f, 0.0f };
	f3 normal = { 0.0f, 0.0f, 0.0f };
	f3 color = { 1.0f, 1.0f, 1.0f };
};
