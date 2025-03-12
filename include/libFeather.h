#pragma once

#include <libFeatherCommon.h>
#include <Core/FeatherWindow.h>

#include <Core/System/GUISystem.h>
#include <Core/System/InputSystem.h>
#include <Core/System/RenderSystem.h>

#ifdef _WINDOWS
struct MonitorInfo {
	HMONITOR hMonitor;
	MONITORINFO monitorInfo;
};

BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData);
void MaximizeConsoleWindowOnMonitor(int monitorIndex);
void MaximizeWindowOnMonitor(HWND hwnd, int monitorIndex);
#endif

class ComponentBase;
class CameraBase;
class Transform;
class PerspectiveCamera;
class OrthogonalCamera;

class libFeather
{
public:
	libFeather();
	~libFeather();

	void Initialize(ui32 width, ui32 height);

	void Run();

	void Terminate();

	Entity* CreateEntity();
	Entity* GetEntity(EntityID id);

	PerspectiveCamera* CreatePerspectiveCamera();

	inline FeatherWindow* GetFeatherWindow() const { return featherWindow; }

	inline void AddOnInitializeCallback(function<void()> callback) { onInitializeCallbacks.push_back(callback); }
	inline void AddOnUpdateCallback(function<void(f32)> callback) { onUpdateCallbacks.push_back(callback); }
	inline void AddOnRenderCallback(function<void(f32)> callback) { onRenderCallbacks.push_back(callback); }
	inline void AddOnTerminateCallback(function<void()> callback) { onTerminateCallbacks.push_back(callback); }

private:
	FeatherWindow* featherWindow = nullptr;

	unordered_map<EntityID, Entity*> entities;
	ui32 nextEntityID = 0;

	unordered_map<ComponentID, ComponentBase*> components;
	ui32 nextComponentID = 0;

	map<string, SystemBase*> systems;

	vector<function<void()>> onInitializeCallbacks;
	vector<function<void(f32)>> onUpdateCallbacks;
	vector<function<void(f32)>> onRenderCallbacks;
	vector<function<void()>> onTerminateCallbacks;

public:
	friend class Feather;
};
