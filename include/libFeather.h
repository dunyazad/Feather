#pragma once

#include <libFeatherCommon.h>
#include <Core/MiniMath.h>
#include <Core/FeatherWindow.h>
#include <Core/Shader.h>

class libFeather
{
public:
	libFeather();
	~libFeather();

	void Initialize();

	void Run();

	void Terminate();

	FeatherWindow* CreateWindow(ui32 width, ui32 height);
	Shader* CreateShader();
	void ReloadFont(float newFontSize);

private:
	bool needFontReload = false;
	f32 fontSize = 18.0f;
	vector<FeatherWindow*> featherWindows;
	vector<Shader*> shaders;
};
