#pragma once

#include <FeatherCommon.h>
#include <File.h>

class EventSystem;
class RenderSystem;
class ImmediateModeRenderSystem;
class GUISystem;

class Entity;
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

	inline void AddOnInitializeCallback(function<void()> callback) { onInitializeCallbacks.push_back(callback); }
	inline void AddOnUpdateCallback(function<void(f32)> callback) { onUpdateCallbacks.push_back(callback); }
	inline void AddOnRenderCallback(function<void(f32)> callback) { onRenderCallbacks.push_back(callback); }
	inline void AddOnTerminateCallback(function<void()> callback) { onTerminateCallbacks.push_back(callback); }

	inline entt::registry& GetRegistry() { return registry; }
	inline entt::dispatcher& GetDispatcher() { return dispatcher; }

	EventSystem* GetEventSystem() { return eventSystem; }
	RenderSystem* GetRenderSystem() { return renderSystem; }
	ImmediateModeRenderSystem* GetImmediateModeRenderSystem() { return immediateModeRenderSystem; }
	GUISystem* GetGUISystem() { return guiSystem; }

	Shader* CreateShader(const string& name, const File& vsFile, const File& gsFile, const File& fsFile);
	Shader* CreateShader(const string& name, const File& vsFile, const File& fsFile);
	Shader* GetShader(const string& name);

private:
	static libFeather* s_instance;

	libFeather();
	~libFeather();

	FeatherWindow* featherWindow = nullptr;

	vector<function<void()>> onInitializeCallbacks;
	vector<function<void(f32)>> onUpdateCallbacks;
	vector<function<void(f32)>> onRenderCallbacks;
	vector<function<void()>> onTerminateCallbacks;

	entt::registry registry;
	entt::dispatcher dispatcher;

	unordered_map<string, Shader*> shaders;

	EventSystem*                  eventSystem;
	RenderSystem*                 renderSystem;
	ImmediateModeRenderSystem*    immediateModeRenderSystem;
	GUISystem*                    guiSystem;
};
