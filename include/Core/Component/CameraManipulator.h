#pragma once

#include <FeatherCommon.h>
#include <Core/Component/ComponentBase.h>

class CameraBase;

class CameraManipulator : public ComponentBase
{
public:
	CameraManipulator(ComponentID id);
	~CameraManipulator();

	inline void SetCamera(CameraBase* camera) { this->camera = camera; }

private:
	CameraBase* camera = nullptr;
};
