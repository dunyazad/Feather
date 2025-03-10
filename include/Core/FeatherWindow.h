#pragma once

#include<libFeatherCommon.h>

class FeatherWindow
{
public:
	FeatherWindow();
	~FeatherWindow();

	GLFWwindow* Initialize(ui32 width, ui32 height);
	void Terminate();

	void Frame();

	inline GLFWwindow* GetGLFWwindow() const { return window; }
	inline ui32 GetWidth() const { return width; }
	inline ui32 GetHeight() const { return height; }

	inline void AddOnInitializeCallback(function<void()> callback) { onInitializeCallbacks.push_back(callback); }
	inline void AddOnUpdateCallback(function<void(f32)> callback) { onUpdateCallbacks.push_back(callback); }
	inline void AddOnRenderCallback(function<void(f32)> callback) { onRenderCallbacks.push_back(callback); }
	inline void AddOnTerminateCallback(function<void()> callback) { onTerminateCallbacks.push_back(callback); }

private:
	GLFWwindow* window = nullptr;
	ui32 width = 1920;
	ui32 height = 1024;

	vector<function<void()>> onInitializeCallbacks;
	vector<function<void(f32)>> onUpdateCallbacks;
	vector<function<void(f32)>> onRenderCallbacks;
	vector<function<void()>> onTerminateCallbacks;
};
