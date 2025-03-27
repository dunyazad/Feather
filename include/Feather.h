#pragma once

#include <FeatherCommon.h>

class Entity;
class FeatherWindow;
class SystemBase;
class ComponentBase;
class CameraBase;
class Transform;
class PerspectiveCamera;
class OrthogonalCamera;

class libFeather
{
public:
	static libFeather& GetStaticInstance()
	{
		static libFeather instance;
		return instance;
	}

	void Initialize(ui32 width, ui32 height);
	void Terminate();

	void Run();

	template<typename T>
	set<T*> GetInstances()
	{
		return GetInstancesOfType<T>(typeid(T));
	}

	template<typename T>
	T* GetFirstInstance()
	{
		auto instances = GetInstancesOfType<T>(typeid(T));
		if (instances.empty()) return nullptr;
		else return *instances.begin();
	}

	template <typename T, typename... Args>
	T* CreateInstance(const string& name = "", Args&&... args)
	{
		auto instance = new T(forward<Args>(args)...);
		instance->SetName(name);
		objectInstances.push_back(instance);
		s_instanceMap[typeid(T)].push_back(instance);
		return instance;
	}

	static unordered_map<type_index, vector<FeatherObject*>>& GetInstanceMap();
	static set<FeatherObject*> GetAllInstances();

	template<typename T>
	static set<T*> GetInstancesOfType(type_index baseType)
	{
		set<T*> instances;

		if (s_instanceMap.find(baseType) != s_instanceMap.end())
		{
			for (auto obj : s_instanceMap[baseType])
			{
				if (auto castedObj = dynamic_cast<T*>(obj))
				{
					instances.insert(castedObj);
				}
			}
		}

		for (const auto& subclass : FeatherObject::GetAllSubclasses(baseType))
		{
			if (s_instanceMap.find(subclass) != s_instanceMap.end())
			{
				for (auto obj : s_instanceMap[subclass])
				{
					if (auto castedObj = dynamic_cast<T*>(obj))
					{
						instances.insert(castedObj);
					}
				}
			}
		}

		return instances;
	}

	inline FeatherWindow* GetFeatherWindow() const { return featherWindow; }

	inline void AddOnInitializeCallback(function<void()> callback) { onInitializeCallbacks.push_back(callback); }
	inline void AddOnUpdateCallback(function<void(f32)> callback) { onUpdateCallbacks.push_back(callback); }
	inline void AddOnRenderCallback(function<void(f32)> callback) { onRenderCallbacks.push_back(callback); }
	inline void AddOnTerminateCallback(function<void()> callback) { onTerminateCallbacks.push_back(callback); }

	inline entt::registry& GetRegistry() { return registry; }
	inline entt::dispatcher& GetDispatcher() { return dispatcher; }

private:
	static libFeather* s_instance;

	libFeather();
	~libFeather();

	FeatherWindow* featherWindow = nullptr;

	static unordered_map<type_index, vector<FeatherObject*>> s_instanceMap;

	vector<FeatherObject*> objectInstances;

	vector<function<void()>> onInitializeCallbacks;
	vector<function<void(f32)>> onUpdateCallbacks;
	vector<function<void(f32)>> onRenderCallbacks;
	vector<function<void()>> onTerminateCallbacks;

	entt::registry registry;
	entt::dispatcher dispatcher;
};
