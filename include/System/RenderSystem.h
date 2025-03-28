#pragma once

#include <FeatherCommon.h>

class FeatherWindow;
class Shader;

class RenderSystem
{
public:
	RenderSystem(FeatherWindow* window);
	~RenderSystem();

	virtual void Initialize();
	virtual void Terminate();
	virtual void Update(ui32 frameNo, f32 timeDelta);

private:
	FeatherWindow* window = nullptr;

	f32 fontSize = 18.0f;
	bool needFontReload = false;
};
