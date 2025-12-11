#pragma once

#include <FeatherCommon.h>

class StatusPanel
{
public:
	StatusPanel();
	~StatusPanel();

	virtual void Render();
//private:
    const ui32 historySize = 50;

    std::vector<f32> fpsHistory;
    ui32 historyOffset = 0;
    f32 accumulatedFPS = 0.0f;
    ui32 frameCount = 0;
    const ui32 updateRate = 10;
    bool vSync = true;

    ui32 mouseX = 0;
    ui32 mouseY = 0;

	f64 usedMemory = 0.0f;
	f64 totalMemory = 0.0f;
};
