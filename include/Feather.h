#pragma once

#include <FeatherCommon.h>
#include <EventDispatcher.h>
#include <EventHandler.h>

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

	inline Entity CreateEntity() { return registry.create(); }
	inline void RemoveEntity(Entity entity) { registry.destroy(entity); }

	template <typename T, typename... Args>
	T& CreateComponent(Entity entity, Args&&... args)
	{
		return registry.emplace<T>(entity, forward<Args>(args)...);
	}

	template <typename T>
	void RemoveComponent(const T& component)
	{
		registry.remove<T>(component);
	}

	inline FeatherWindow* GetFeatherWindow() const { return featherWindow; }

	inline uv_loop_t* GetLoop() const { return loop; }

	inline entt::registry& GetRegistry() { return registry; }

private:
	static libFeather* s_instance;

	libFeather();
	~libFeather();

	uv_loop_t* loop = nullptr;
	uv_poll_t glfw_poll;
	uv_idle_t idle_handle;

	FeatherWindow* featherWindow = nullptr;

	entt::registry registry;
};
