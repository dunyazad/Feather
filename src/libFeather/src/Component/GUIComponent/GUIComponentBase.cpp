#include <Component/GUIComponent/GUIComponentBase.h>

GUIComponentBase::GUIComponentBase(ComponentID id)
	: RegisterDerivation<GUIComponentBase, ComponentBase>(id)
{
}

GUIComponentBase::~GUIComponentBase()
{
}
