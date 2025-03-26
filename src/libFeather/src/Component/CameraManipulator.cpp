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
	SubscribeEvent(EventType::MousePosition);
	SubscribeEvent(EventType::MouseButtonPress);
	SubscribeEvent(EventType::MouseButtonRelease);
	SubscribeEvent(EventType::MouseWheel);
	SubscribeEvent(EventType::KeyPress);
	SubscribeEvent(EventType::KeyRelease);
}

CameraManipulatorOrbit::~CameraManipulatorOrbit()
{
}

void CameraManipulatorOrbit::OnEvent(const Event& event)
{
	CameraManipulatorBase::OnEvent(event);

	switch (event.type)
	{
	case EventType::MousePosition:
		OnMousePosition(event);
		break;
	case EventType::MouseButtonPress:
		OnMouseButtonPress(event);
		break;
	case EventType::MouseButtonRelease:
		OnMouseButtonRelease(event);
		break;
	case EventType::MouseWheel:
		OnMouseWheel(event);
		break;
	case EventType::KeyPress:
		OnKeyPress(event);
		break;
	case EventType::KeyRelease:
		OnKeyRelease(event);
		break;
	default:
		break;
	}
}

void CameraManipulatorOrbit::OnMousePosition(const Event& event)
{
	if (nullptr == camera) return;

	if (isRButtonPressed)
	{
		auto hDelta = event.mousePositionEvent.xpos - lastMousePositionX;
		azimuth -= hDelta * mouseSensitivity;
		if (azimuth < 0.0f) azimuth += 360.0f;
		else if (azimuth >= 360.0f) azimuth -= 360.0f;

		auto vDelta = event.mousePositionEvent.ypos - lastMousePositionY;
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
		auto hDelta = event.mousePositionEvent.xpos - lastMousePositionX;
		auto hPanning = hDelta * mousePanningSensitivity;

		auto vDelta = event.mousePositionEvent.ypos - lastMousePositionY;
		auto vPanning = vDelta * mousePanningSensitivity;

		auto viewMatrix = camera->GetViewMatrix();
		MiniMath::V3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0), viewMatrix.at(2, 0));
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

	lastMousePositionX = event.mousePositionEvent.xpos;
	lastMousePositionY = event.mousePositionEvent.ypos;
}

void CameraManipulatorOrbit::OnMouseButtonPress(const Event& event)
{
	if (nullptr == camera) return;

	if (0 == event.mouseButtonEvent.button)
	{
		isLButtonPressed = true;
	}
	else if (1 == event.mouseButtonEvent.button)
	{
		isRButtonPressed = true;
	}
	else if (2 == event.mouseButtonEvent.button)
	{
		isMButtonPressed = true;
	}
	//alog("Button Press : %d - x : %.2f, y : %.2f\n",
	//	event.mouseButtonEvent.button,
	//	event.mouseButtonEvent.xpos,
	//	event.mouseButtonEvent.ypos);
}

void CameraManipulatorOrbit::OnMouseButtonRelease(const Event& event)
{
	if (nullptr == camera) return;

	if (0 == event.mouseButtonEvent.button)
	{
		isLButtonPressed = false;
	}
	else if (1 == event.mouseButtonEvent.button)
	{
		isRButtonPressed = false;
	}
	else if (2 == event.mouseButtonEvent.button)
	{
		isMButtonPressed = false;
	}
	//alog("Button Release : %d - x : %.2f, y : %.2f\n",
	//	event.mouseButtonEvent.button,
	//	event.mouseButtonEvent.xpos,
	//	event.mouseButtonEvent.ypos);
}

void CameraManipulatorOrbit::OnMouseWheel(const Event& event)
{
	if (nullptr == camera) return;

	if (0 != pressedKeys.count(GLFW_KEY_LEFT_SHIFT) || 0 != pressedKeys.count(GLFW_KEY_RIGHT_SHIFT))
	{
		auto perspectiveCamera = dynamic_cast<PerspectiveCamera*>(camera);
		if (nullptr != perspectiveCamera)
		{
			f32 fovy = perspectiveCamera->GetFOVY() * RAD2DEG;

			if (0 > event.mouseWheelEvent.yoffset)
			{
				fovy += 1.0f;
			}
			else if (0 < event.mouseWheelEvent.yoffset)
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
		if (0 > event.mouseWheelEvent.yoffset)
		{
			radius *= 1.1f;
		}
		else if (0 < event.mouseWheelEvent.yoffset)
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
}

void CameraManipulatorOrbit::OnKeyPress(const Event& event)
{
	pressedKeys.insert(event.keyEvent.keyCode);

	if (nullptr == camera) return;

	if (GLFW_KEY_LEFT == event.keyEvent.keyCode)
	{
		auto hDelta = 1;
		auto hPanning = hDelta * mouseSensitivity;

		auto vDelta = 0;
		auto vPanning = vDelta * mouseSensitivity;

		auto viewMatrix = camera->GetViewMatrix();
		MiniMath::V3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0), viewMatrix.at(2, 0));
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
	else if (GLFW_KEY_RIGHT == event.keyEvent.keyCode)
	{
		auto hDelta = -1;
		auto hPanning = hDelta * mouseSensitivity;

		auto vDelta = 0;
		auto vPanning = vDelta * mouseSensitivity;

		auto viewMatrix = camera->GetViewMatrix();
		MiniMath::V3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0), viewMatrix.at(2, 0));
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
	else if (GLFW_KEY_L == event.keyEvent.keyCode)
	{
		camera->Reset();
	}
	else if (GLFW_KEY_INSERT == event.keyEvent.keyCode)
	{
		camera->PushCameraHistory();
	}
	else if (GLFW_KEY_DELETE == event.keyEvent.keyCode)
	{
		camera->PopCameraHistory();
	}
	else if (GLFW_KEY_PAGE_UP == event.keyEvent.keyCode)
	{
		camera->JumpToNextCameraHistory();
	}
	else if (GLFW_KEY_PAGE_DOWN == event.keyEvent.keyCode)
	{
		camera->JumpToPreviousCameraHistory();
	}
	else if (
		GLFW_KEY_0 <= event.keyEvent.keyCode &&
		GLFW_KEY_9 >= event.keyEvent.keyCode)
	{
		camera->JumpCameraHistory(event.keyEvent.keyCode - GLFW_KEY_0);
	}
}

void CameraManipulatorOrbit::OnKeyRelease(const Event& event)
{
	pressedKeys.erase(event.keyEvent.keyCode);

	if (nullptr == camera) return;
}

CameraManipulatorTrackball::CameraManipulatorTrackball()
	: RegisterDerivation<CameraManipulatorTrackball, CameraManipulatorBase>(),
	radius(10.0f),
	mouseSensitivity(0.005f),
	mousePanningSensitivity(0.01f),
	isLButtonPressed(false),
	isMButtonPressed(false),
	isRButtonPressed(false)
{
	SubscribeEvent(EventType::MousePosition);
	SubscribeEvent(EventType::MouseButtonPress);
	SubscribeEvent(EventType::MouseButtonRelease);
	SubscribeEvent(EventType::MouseWheel);
	SubscribeEvent(EventType::KeyPress);
	SubscribeEvent(EventType::KeyRelease);
}

CameraManipulatorTrackball::~CameraManipulatorTrackball() {}

void CameraManipulatorTrackball::OnEvent(const Event& event)
{
	CameraManipulatorBase::OnEvent(event);

	switch (event.type) {
	case EventType::MousePosition:
		OnMousePosition(event);
		break;
	case EventType::MouseButtonPress:
		OnMouseButtonPress(event);
		break;
	case EventType::MouseButtonRelease:
		OnMouseButtonRelease(event);
		break;
	case EventType::MouseWheel:
		OnMouseWheel(event);
		break;
	case EventType::KeyPress:
		OnKeyPress(event);
		break;
	case EventType::KeyRelease:
		OnKeyRelease(event);
		break;
	default:
		break;
	}
}

void CameraManipulatorTrackball::OnMousePosition(const Event& event)
{
	if (nullptr == camera) return;

	float dx = event.mousePositionEvent.xpos - lastMousePositionX;
	float dy = event.mousePositionEvent.ypos - lastMousePositionY;

	if (isRButtonPressed)
	{
		float angleX = -dx * mouseSensitivity;
		float angleY = dy * mouseSensitivity;

		MiniMath::V3 eye = camera->GetEye();
		MiniMath::V3 target = camera->GetTarget();
		MiniMath::V3 up = camera->GetUp();

		MiniMath::V3 viewDir = MiniMath::normalize(eye - target);
		MiniMath::V3 right = MiniMath::normalize(MiniMath::cross(viewDir, up));

		MiniMath::Quaternion rotX = MiniMath::Quaternion(angleX, up);
		MiniMath::Quaternion rotY = MiniMath::Quaternion(angleY, right);

		MiniMath::V3 rotatedViewDir = MiniMath::rotate(viewDir, rotX * rotY);

		MiniMath::V3 newEye = target + rotatedViewDir * radius;

		MiniMath::V3 newUp = MiniMath::normalize(MiniMath::cross(right, rotatedViewDir));

		camera->SetEye(newEye);
		camera->SetUp(newUp);
	}

	if (isMButtonPressed) {
		float panX = -dx * mousePanningSensitivity;
		float panY = dy * mousePanningSensitivity;

		MiniMath::M4 viewMatrix = camera->GetViewMatrix();
		MiniMath::V3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0), viewMatrix.at(2, 0));
		MiniMath::V3 up(viewMatrix.at(0, 1), viewMatrix.at(1, 1), viewMatrix.at(2, 1));

		MiniMath::V3 eye = camera->GetEye();
		MiniMath::V3 target = camera->GetTarget();

		target += right * panX + up * panY;
		eye += right * panX + up * panY;

		camera->SetTarget(target);
		camera->SetEye(eye);
	}

	lastMousePositionX = event.mousePositionEvent.xpos;
	lastMousePositionY = event.mousePositionEvent.ypos;
}

void CameraManipulatorTrackball::OnMouseButtonPress(const Event& event)
{
	if (nullptr == camera) return;

	if (event.mouseButtonEvent.button == 0)
	{
		isLButtonPressed = true;
	}
	else if (event.mouseButtonEvent.button == 1)
	{
		isRButtonPressed = true;
	}
	else if (event.mouseButtonEvent.button == 2)
	{
		isMButtonPressed = true;
	}

	lastMousePositionX = event.mouseButtonEvent.xpos;
	lastMousePositionY = event.mouseButtonEvent.ypos;
}

void CameraManipulatorTrackball::OnMouseButtonRelease(const Event& event)
{
	if (nullptr == camera) return;

	if (event.mouseButtonEvent.button == 0)
	{
		isLButtonPressed = false;
	}
	else if (event.mouseButtonEvent.button == 1)
	{
		isRButtonPressed = false;
	}
	else if (event.mouseButtonEvent.button == 2)
	{
		isMButtonPressed = false;
	}
}

void CameraManipulatorTrackball::OnMouseWheel(const Event& event)
{
	if (nullptr == camera) return;

	if (0 != pressedKeys.count(GLFW_KEY_LEFT_SHIFT) || 0 != pressedKeys.count(GLFW_KEY_RIGHT_SHIFT))
	{
		auto perspectiveCamera = dynamic_cast<PerspectiveCamera*>(camera);
		if (nullptr != perspectiveCamera)
		{
			f32 fovy = perspectiveCamera->GetFOVY() * RAD2DEG;

			if (0 > event.mouseWheelEvent.yoffset)
			{
				fovy += 1.0f;
			}
			else if (0 < event.mouseWheelEvent.yoffset)
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
		if (0 > event.mouseWheelEvent.yoffset)
		{
			radius *= 1.1f;
		}
		else if (0 < event.mouseWheelEvent.yoffset)
		{
			radius *= 0.9f;
		}

		auto eye = camera->GetEye();
		auto target = camera->GetTarget();

		MiniMath::V3 viewDir = MiniMath::normalize(eye - target);
		camera->SetEye(target + viewDir * radius);
	}
}

void CameraManipulatorTrackball::OnKeyPress(const Event& event)
{
	pressedKeys.insert(event.keyEvent.keyCode);

	if (nullptr == camera) return;

	MiniMath::V3 eye = camera->GetEye();
	MiniMath::V3 target = camera->GetTarget();
	MiniMath::V3 up = camera->GetUp();
	MiniMath::V3 viewDir = MiniMath::normalize(target - eye);
	MiniMath::V3 right = MiniMath::normalize(MiniMath::cross(up, viewDir));

	float moveStep = 0.2f;

	if (event.keyEvent.keyCode == GLFW_KEY_W)
	{
		eye += viewDir * moveStep;
		target += viewDir * moveStep;
	}
	else if (event.keyEvent.keyCode == GLFW_KEY_S)
	{
		eye -= viewDir * moveStep;
		target -= viewDir * moveStep;
	}
	else if (event.keyEvent.keyCode == GLFW_KEY_A)
	{
		eye -= right * moveStep;
		target -= right * moveStep;
	}
	else if (event.keyEvent.keyCode == GLFW_KEY_D)
	{
		eye += right * moveStep;
		target += right * moveStep;
	}
	else if (event.keyEvent.keyCode == GLFW_KEY_R)
	{
		camera->Reset();
	}

	camera->SetEye(eye);
	camera->SetTarget(target);
}

void CameraManipulatorTrackball::OnKeyRelease(const Event& event)
{
	pressedKeys.erase(event.keyEvent.keyCode);
}
