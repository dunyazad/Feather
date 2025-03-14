#include <Component/ComponentBase.h>

ComponentBase::ComponentBase(ComponentID id)
	: RegisterDerivation<ComponentBase, FeatherObject>(), id(id)
{
}

ComponentBase::~ComponentBase()
{
}

void ComponentBase::Update(ui32 frameNo, f32 timeDelta)
{

}
