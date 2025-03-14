#pragma once

#include <vector>

#define WIN32_LEAN_AND_MEAN
#define NO_MINMAX
#define NO_BYTE
#include <windows.h>
#include <shellapi.h>

#ifdef _WINDOWS
struct MonitorInfo {
	HMONITOR hMonitor;
	MONITORINFO monitorInfo;
};

BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData);
void MaximizeConsoleWindowOnMonitor(int monitorIndex);
void MaximizeWindowOnMonitor(HWND hwnd, int monitorIndex);
#endif
