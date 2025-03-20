#pragma once

#include <System/SystemBase.h>

class Shader;

class RenderSystem
{
public:
	RenderSystem(FeatherWindow* window);
	~RenderSystem();

	virtual void Initialize();
	virtual void Terminate();
	virtual void Update(ui32 frameNo, f32 timeDelta);

	Shader* CreateShader();

private:
	f32 fontSize = 18.0f;
	bool needFontReload = false;

	vector<Shader*> shaders;
};
