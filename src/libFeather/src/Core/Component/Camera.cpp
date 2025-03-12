#include <Core/Component/Camera.h>

CameraBase::CameraBase(ComponentID id)
	: ComponentBase(id)
{
}

CameraBase::~CameraBase()
{
}

PerspectiveCamera::PerspectiveCamera(ComponentID id)
	: CameraBase(id)
{
}

PerspectiveCamera::~PerspectiveCamera()
{
}

OrthogonalCamera::OrthogonalCamera(ComponentID id)
	: CameraBase(id)
{
}

OrthogonalCamera::~OrthogonalCamera()
{
}
