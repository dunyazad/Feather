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

    vector<f32> fpsHistory;
    ui32 historyOffset = 0;
    f32 accumulatedFPS = 0.0f;
    ui32 frameCount = 0;
    const ui32 updateRate = 10;
    bool vSync = true;

    ui32 mouseX = 0;
    ui32 mouseY = 0;
};
