#include <Core/Component/CameraManipulator.h>
#include <Core/Component/Camera.h>
#include <Feather.h>

CameraManipulatorBase::CameraManipulatorBase(ComponentID id)
	: ComponentBase(id)
{
}

CameraManipulatorOrbit::CameraManipulatorOrbit(ComponentID id)
	: CameraManipulatorBase(id)
{
	AddEventHandler(EventType::MousePosition, [&](const Event& event) {
		if (nullptr == camera) return;

		if (isRButtonPressed)
		{
			auto hDelta = event.parameters.mousePosition.xpos - lastMousePositionX;
			azimuth -= hDelta * mouseSensitivity;
			if (azimuth < 0.0f) azimuth += 360.0f;
			else if (azimuth >= 360.0f) azimuth -= 360.0f;

			auto vDelta = event.parameters.mousePosition.ypos - lastMousePositionY;
			elevation += vDelta * mouseSensitivity;
			if (elevation <= -90.0f) elevation = -90.0f + FLT_EPSILON;
			else if (elevation >= 90.0f) elevation = 90.0f - FLT_EPSILON;

			alog("xDelta : %f, yDelta : %f\n", hDelta, vDelta);

			UpdateCamera();
		}

		lastMousePositionX = event.parameters.mousePosition.xpos;
		lastMousePositionY = event.parameters.mousePosition.ypos;
		});

	AddEventHandler(EventType::MouseButtonPress, [&](const Event& event) {
		if (0 == event.parameters.mouseButton.button)
		{
			isLButtonPressed = true;
			lastLButtonPositionX = event.parameters.mouseButton.xpos;
			lastLButtonPositionY = event.parameters.mouseButton.xpos;
		}
		else if (1 == event.parameters.mouseButton.button)
		{
			isRButtonPressed = true;
			lastRButtonPositionX = event.parameters.mouseButton.xpos;
			lastRButtonPositionY = event.parameters.mouseButton.xpos;
		}
		alog("Button Press : %d - x : %.2f, y : %.2f\n",
			event.parameters.mouseButton.button,
			event.parameters.mouseButton.xpos,
			event.parameters.mouseButton.ypos);
		});

	AddEventHandler(EventType::MouseButtonRelease, [&](const Event& event) {
		if (0 == event.parameters.mouseButton.button)
		{
			isLButtonPressed = false;
			lastLButtonPositionX = UINT32_MAX;
			lastLButtonPositionY = UINT32_MAX;
		}
		else if (1 == event.parameters.mouseButton.button)
		{
			isRButtonPressed = false;
			lastRButtonPositionX = UINT32_MAX;
			lastRButtonPositionY = UINT32_MAX;
		}
		alog("Button Release : %d - x : %.2f, y : %.2f\n",
			event.parameters.mouseButton.button,
			event.parameters.mouseButton.xpos,
			event.parameters.mouseButton.ypos);
		});

	AddEventHandler(EventType::MouseWheel, [&](const Event& event) {
		if (nullptr == camera) return;

		if (0 > event.parameters.mouseWheel.yoffset)
		{
			radius *= 1.1f;
		}
		else if (0 < event.parameters.mouseWheel.yoffset)
		{
			radius *= 0.9f;
		}

		UpdateCamera();
		});

	AddEventHandler(EventType::KeyPress, [&](const Event& event) {
		if (nullptr == camera) return;

		if (GLFW_KEY_LEFT == event.parameters.key.keyCode)
		{
			auto entity = Feather::GetInstance().GetEntity(0);
			camera->GetEye().x -= 1.0f;
		}
		else if (GLFW_KEY_RIGHT == event.parameters.key.keyCode)
		{
			auto entity = Feather::GetInstance().GetEntity(0);
			camera->GetEye().x += 1.0f;
		}
		});
}

CameraManipulatorOrbit::~CameraManipulatorOrbit()
{
}

void CameraManipulatorOrbit::UpdateCamera()
{
	auto target = camera->GetTarget();

	MiniMath::V3 eye(
		target.x + radius * cos(elevation * DEG2RAD) * sin(azimuth * DEG2RAD),
		target.y + radius * sin(elevation * DEG2RAD),
		target.z + radius * cos(elevation * DEG2RAD) * cos(azimuth * DEG2RAD));

	camera->SetEye(eye);
}