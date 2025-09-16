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

	inline bool IsEnabled() const { return bEnabled; }
	inline void SetEnable(bool enable) { bEnabled = enable; }
	inline void ToggleEnable() { bEnabled = !bEnabled; }

private:
	FeatherWindow* window = nullptr;

	bool bEnabled = true;
};
