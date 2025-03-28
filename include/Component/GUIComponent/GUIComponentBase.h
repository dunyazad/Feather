#pragma once

#include <FeatherCommon.h>
#include <Component/ComponentBase.h>

class GUIComponentBase
{
public:
	GUIComponentBase();
	virtual ~GUIComponentBase();

	virtual void Render() = 0;
private:
};
