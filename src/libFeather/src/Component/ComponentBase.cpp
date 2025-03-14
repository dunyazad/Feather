#include <Component/ComponentBase.h>

ComponentBase::ComponentBase(ComponentID id)
	: id(id)
{
}

ComponentBase::~ComponentBase()
{
}

void ComponentBase::Update(ui32 frameNo, f32 timeDelta)
{

}
