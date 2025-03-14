#pragma once

#include <System/SystemBase.h>

class ImmediateModeRenderSystem : public SystemBase
{
public:
	ImmediateModeRenderSystem(FeatherWindow* window);
	~ImmediateModeRenderSystem();

	virtual void Initialize() override;
	virtual void Terminate() override;
	virtual void Update(ui32 frameNo, f32 timeDelta) override;

private:
};
