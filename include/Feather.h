#pragma once

#include <FeatherCommon.h>

#ifdef _WINDOWS
struct MonitorInfo {
	HMONITOR hMonitor;
	MONITORINFO monitorInfo;
};

BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData);
void MaximizeConsoleWindowOnMonitor(int monitorIndex);
void MaximizeWindowOnMonitor(HWND hwnd, int monitorIndex);
#endif

class Entity;
class FeatherWindow;
class SystemBase;
class ComponentBase;
class CameraBase;
class Transform;
class PerspectiveCamera;
class OrthogonalCamera;

class Feather
{
public:
	static Feather& GetInstance()
	{
		static Feather instance;
		return instance;
	}

	void Initialize(ui32 width, ui32 height);

	void Run();

	void Terminate();

	Entity* CreateEntity(const string& name = "");
	Entity* GetEntity(EntityID id);

	PerspectiveCamera* CreatePerspectiveCamera();

	ComponentBase* GetComponent(ComponentID id);

	inline const vector<ComponentBase*>& GetComponents() { return components; }
	const vector<ComponentID>& GetComponentsByEntityID(EntityID entityID) { return entityComponentMapping[entityID]; }
	const vector<ui32>& GetComponentIDsByTypeIndex(const type_index& typeIndex);

	template <typename T>
	T* GetSystem()
	{
		if (0 == systems.count(typeid(T))) return nullptr;
		else return dynamic_cast<T*>(systems[typeid(T)]);
	}

	inline FeatherWindow* GetFeatherWindow() const { return featherWindow; }

	inline void AddOnInitializeCallback(function<void()> callback) { onInitializeCallbacks.push_back(callback); }
	inline void AddOnUpdateCallback(function<void(f32)> callback) { onUpdateCallbacks.push_back(callback); }
	inline void AddOnRenderCallback(function<void(f32)> callback) { onRenderCallbacks.push_back(callback); }
	inline void AddOnTerminateCallback(function<void()> callback) { onTerminateCallbacks.push_back(callback); }

private:
	static Feather* s_instance;

	Feather();
	~Feather();

	FeatherWindow* featherWindow = nullptr;

	unordered_map<EntityID, Entity*> entities;
	ui32 nextEntityID = 0;

	vector<ComponentBase*> components;
	unordered_map<std::type_index, vector<ui32>> typeComponentMapping;
	unordered_map<ComponentID, ui32> idComponentMapping;
	ui32 nextComponentID = 0;

	unordered_map<EntityID, vector<ComponentID>> entityComponentMapping;

	map<type_index, SystemBase*> systems;

	vector<function<void()>> onInitializeCallbacks;
	vector<function<void(f32)>> onUpdateCallbacks;
	vector<function<void(f32)>> onRenderCallbacks;
	vector<function<void()>> onTerminateCallbacks;
};
