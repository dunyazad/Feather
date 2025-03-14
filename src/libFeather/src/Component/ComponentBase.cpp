#include <Component/ComponentBase.h>

ComponentID ComponentBase::s_id = 0;

ComponentBase::ComponentBase()
	: RegisterDerivation<ComponentBase, FeatherObject>(), id(s_id++)
{
}

ComponentBase::~ComponentBase()
{
}

void ComponentBase::Update(ui32 frameNo, f32 timeDelta)
{

}
