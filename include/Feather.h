#pragma once

#include <FeatherCommon.h>
#include <File.h>
#include <VisualDebugging.h>
#include <IVisualDebugging.h>
#include <Component/EventCallback.h>
#include <FeatherWindow.h>

class InputSystem;
class EventSystem;
class RenderSystem;
class ImmediateModeRenderSystem;
class GUISystem;

class FeatherWindow;
class Shader;

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

	inline FeatherWindow* GetFeatherWindow() const { return featherWindow; }
	inline HWND GetHWND() { return glfwGetWin32Window(featherWindow->GetGLFWwindow()); };

	inline void AddOnInitializeCallback(std::function<void()> callback) { onInitializeCallbacks.push_back(callback); }
	inline void AddOnUpdateCallback(std::function<void(f32)> callback) { onUpdateCallbacks.push_back(callback); }
	inline void AddOnRenderCallback(std::function<void(f32)> callback) { onRenderCallbacks.push_back(callback); }
	inline void AddOnTerminateCallback(std::function<void()> callback) { onTerminateCallbacks.push_back(callback); }

	inline Registry& GetRegistry() { return registry; }
	inline Dispatcher& GetDispatcher() { return dispatcher; }

	EventSystem* GetEventSystem() { return eventSystem; }
	RenderSystem* GetRenderSystem() { return renderSystem; }
	ImmediateModeRenderSystem* GetImmediateModeRenderSystem() { return immediateModeRenderSystem; }
	GUISystem* GetGUISystem() { return guiSystem; }

	Entity CreateEntity(const std::string& name);
	Entity GetEntityByName(const std::string& name);
	template<typename T>
	Entity GetEntityByComponent(T* t)
	{
		auto view = registry.view<T>();

		for (auto entity : view)
		{
			const auto& comp = view.get<T>(entity);
			if (&comp == t)
			{
				return entity;
			}
		}

		return InvalidEntity;
	}

	const std::string& GetEntityName(Entity entity);
	void RemoveEntity(const std::string& name);
	void RemoveEntity(Entity entity);

	template<typename T>
	T* GetComponent(Entity entity)
	{
		if (false == registry.all_of<T>(entity))
			return nullptr;
		else
			return &registry.get<T>(entity);
	}

	template<typename T, typename... Args>
	T* CreateComponent(Entity entity, Args&&... args) {
		if (registry.all_of<T>(entity)) {
			return &registry.get<T>(entity);
		}
		return &(registry.emplace<T>(entity, std::forward<Args>(args)...));
	}

	template<typename T>
	EventCallback<T>& GetEventCallback(Entity entity)
	{
		assert(registry.all_of<EventCallback<T>>(entity) && "Entity does not have the requested event callback.");
		return registry.get<EventCallback<T>>(entity);
	}

	template<typename T, typename... Args>
	EventCallback<T>& CreateEventCallback(Entity entity, Args&&... args) {
		if (registry.all_of<EventCallback<T>>(entity)) {
			return registry.get<EventCallback<T>>(entity);
		}
		return registry.emplace<EventCallback<T>>(entity, entity, std::forward<Args>(args)...);
	}

	template<typename T>
	void RemoveEventCallback(Entity entity)
	{
		if (registry.all_of<EventCallback<T>>(entity))
		{
			registry.remove<EventCallback<T>>(entity);
		}
	}

	Shader* CreateShader(const std::string& name, const File& vsFile, const File& gsFile, const File& fsFile);
	Shader* CreateShader(const std::string& name, const File& vsFile, const File& fsFile);
	Shader* GetShader(const std::string& name);

	inline const glm::vec4& GetClearColor() const { return clearColor; }
	inline void SetClearColor(const glm::vec4& color) { clearColor = color; }

private:
	libFeather();
	~libFeather();

	FeatherWindow* featherWindow = nullptr;
	unsigned int consoleWindowIndex = 2;
	unsigned int mainWindowIndex = 1;

	std::vector<std::function<void()>> onInitializeCallbacks;
	std::vector<std::function<void(f32)>> onUpdateCallbacks;
	std::vector<std::function<void(f32)>> onRenderCallbacks;
	std::vector<std::function<void()>> onTerminateCallbacks;

	Registry registry;
	Dispatcher dispatcher;

	std::unordered_map<std::string, Entity> nameEntityMapping;
	std::unordered_map<Entity, std::string> entityNameMapping;
	std::unordered_map<std::string, Shader*> shaders;

	InputSystem*					inputSystem = nullptr;
	EventSystem*					eventSystem = nullptr;
	RenderSystem*					renderSystem = nullptr;
	ImmediateModeRenderSystem*		immediateModeRenderSystem = nullptr;
	GUISystem*						guiSystem = nullptr;

	glm::vec4 clearColor = Color::slategray();
};
