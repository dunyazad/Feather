#include <Component/CameraManipulator.h>
#include <Component/Camera.h>
#include <Feather.h>

CameraManipulatorBase::CameraManipulatorBase()
	: RegisterDerivation<CameraManipulatorBase, ComponentBase>()
{
}

CameraManipulatorOrbit::CameraManipulatorOrbit()
	: RegisterDerivation<CameraManipulatorOrbit, CameraManipulatorBase>()
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
			if (elevation <= -90.0f) elevation = -89.9f;
			else if (elevation >= 90.0f) elevation = 89.9f;

			auto target = camera->GetTarget();

			MiniMath::V3 eye(
				target.x + radius * cos(elevation * DEG2RAD) * sin(azimuth * DEG2RAD),
				target.y + radius * sin(elevation * DEG2RAD),
				target.z + radius * cos(elevation * DEG2RAD) * cos(azimuth * DEG2RAD));

			camera->SetEye(eye);
		}

		if (isMButtonPressed)
		{
			auto hDelta = event.parameters.mousePosition.xpos - lastMousePositionX;
			auto hPanning = hDelta * mouseSensitivity;

			auto vDelta = event.parameters.mousePosition.ypos - lastMousePositionY;
			auto vPanning = vDelta * mouseSensitivity;

			auto viewMatrix = camera->GetViewMatrix();
			MiniMath::V3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0),viewMatrix.at(2, 0));
			MiniMath::V3 up(viewMatrix.at(0, 1), viewMatrix.at(1, 1), viewMatrix.at(2, 1));

			auto target = camera->GetTarget();
			auto eye = camera->GetEye();

			if (hDelta != 0)
			{
				target -= right * (hPanning * radius * 0.01f);
				eye -= right * (hPanning * radius * 0.01f);
			}

			if (vDelta != 0)
			{
				target += up * (vPanning * radius * 0.01f);
				eye += up * (vPanning * radius * 0.01f);
			}

			camera->SetTarget(target);
			camera->SetEye(eye);
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
		else if (2 == event.parameters.mouseButton.button)
		{
			isMButtonPressed = true;
			lastMButtonPositionX = event.parameters.mouseButton.xpos;
			lastMButtonPositionY = event.parameters.mouseButton.xpos;
		}
		//alog("Button Press : %d - x : %.2f, y : %.2f\n",
		//	event.parameters.mouseButton.button,
		//	event.parameters.mouseButton.xpos,
		//	event.parameters.mouseButton.ypos);
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
		else if (2 == event.parameters.mouseButton.button)
		{
			isMButtonPressed = false;
			lastMButtonPositionX = UINT32_MAX;
			lastMButtonPositionY = UINT32_MAX;
		}
		//alog("Button Release : %d - x : %.2f, y : %.2f\n",
		//	event.parameters.mouseButton.button,
		//	event.parameters.mouseButton.xpos,
		//	event.parameters.mouseButton.ypos);
		});

	AddEventHandler(EventType::MouseWheel, [&](const Event& event) {
		if (nullptr == camera) return;

		if (0 != pressedKeys.count(GLFW_KEY_LEFT_SHIFT) || 0 != pressedKeys.count(GLFW_KEY_RIGHT_SHIFT))
		{
			auto perspectiveCamera = dynamic_cast<PerspectiveCamera*>(camera);
			if (nullptr != perspectiveCamera)
			{
				f32 fovy = perspectiveCamera->GetFOVY() * RAD2DEG;

				if (0 > event.parameters.mouseWheel.yoffset)
				{
					fovy += 1.0f;
				}
				else if (0 < event.parameters.mouseWheel.yoffset)
				{
					fovy -= 1.0f;
				}
				
				if (1 > fovy) fovy = 1.0f;
				if (90 < fovy) fovy = 90.0f;

				perspectiveCamera->SetFOVY(fovy * DEG2RAD);
			}
		}
		else
		{
			if (0 > event.parameters.mouseWheel.yoffset)
			{
				radius *= 1.1f;
			}
			else if (0 < event.parameters.mouseWheel.yoffset)
			{
				radius *= 0.9f;
			}

			auto target = camera->GetTarget();

			MiniMath::V3 eye(
				target.x + radius * cos(elevation * DEG2RAD) * sin(azimuth * DEG2RAD),
				target.y + radius * sin(elevation * DEG2RAD),
				target.z + radius * cos(elevation * DEG2RAD) * cos(azimuth * DEG2RAD));

			camera->SetEye(eye);
		}
		});

	AddEventHandler(EventType::KeyPress, [&](const Event& event) {
		pressedKeys.insert(event.parameters.key.keyCode);

		//alog("%d\n", event.parameters.key.keyCode);

		if (nullptr == camera) return;

		if (GLFW_KEY_LEFT == event.parameters.key.keyCode)
		{
			camera->GetEye().x -= 1.0f;
		}
		else if (GLFW_KEY_RIGHT == event.parameters.key.keyCode)
		{
			camera->GetEye().x += 1.0f;
		}
		});

	AddEventHandler(EventType::KeyRelease, [&](const Event& event) {
		pressedKeys.erase(event.parameters.key.keyCode);
		
		if (nullptr == camera) return;
		
		});
}

CameraManipulatorOrbit::~CameraManipulatorOrbit()
{
}
