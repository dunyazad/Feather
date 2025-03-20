#pragma once

#include <System/SystemBase.h>

class ImmediateModeRenderSystem
{
public:
	ImmediateModeRenderSystem(FeatherWindow* window);
	~ImmediateModeRenderSystem();

	virtual void Initialize();
	virtual void Terminate();
	virtual void Update(ui32 frameNo, f32 timeDelta);

private:
};
