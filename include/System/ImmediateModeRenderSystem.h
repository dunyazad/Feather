#pragma once

#include <FeatherCommon.h>

class FeatherWindow;

class ImmediateModeRenderSystem
{
public:
	ImmediateModeRenderSystem(FeatherWindow* window);
	~ImmediateModeRenderSystem();

	virtual void Initialize();
	virtual void Terminate();
	virtual void Update(ui32 frameNo, f32 timeDelta);

private:
	FeatherWindow* window = nullptr;
};
