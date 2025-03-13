#pragma once

#include <FeatherCommon.h>
#include <Core/Component/EventReceiver.h>

class CameraBase;

class CameraManipulator : public EventReceiver
{
public:
	CameraManipulator(ComponentID id);
	~CameraManipulator();

	inline void SetCamera(CameraBase* camera) { this->camera = camera; }

private:
	CameraBase* camera = nullptr;
};
