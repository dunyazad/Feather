#pragma once

#include<FeatherCommon.h>

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

private:
	GLFWwindow* window = nullptr;
	ui32 width = 1920;
	ui32 height = 1024;
};
