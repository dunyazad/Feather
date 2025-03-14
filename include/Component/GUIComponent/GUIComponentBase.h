#pragma once

#include <FeatherCommon.h>
#include <Component/ComponentBase.h>

class GUIComponentBase : public RegisterDerivation<GUIComponentBase, ComponentBase>
{
public:
	GUIComponentBase(ComponentID id);
	virtual ~GUIComponentBase();

private:
};
