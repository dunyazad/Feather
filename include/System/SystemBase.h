#pragma once

#include<libFeatherCommon.h>

class FeatherWindow;

class SystemBase
{
public:
	SystemBase(FeatherWindow* window);
	~SystemBase();

	virtual void Initialize() = 0;
	virtual void Terminate() = 0;
	virtual void Update(ui32 frameNo, f32 timeDelta) = 0;

protected:
	FeatherWindow* window = nullptr;
};
