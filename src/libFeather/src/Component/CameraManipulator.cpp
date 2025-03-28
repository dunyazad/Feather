#include <Component/CameraManipulator.h>
#include <Component/Camera.h>
#include <Component/EventCallback.h>
#include <Feather.h>

CameraManipulatorBase::CameraManipulatorBase()
{
}

CameraManipulatorOrbit::CameraManipulatorOrbit()
{
}

CameraManipulatorOrbit::~CameraManipulatorOrbit()
{
}

//void CameraManipulatorOrbit::OnKey(const KeyEvent& event)
//{
//	pressedKeys.insert(event.keyCode);
//
//	if (nullptr == camera) return;
//
//	if (GLFW_KEY_LEFT == event.keyEvent.keyCode)
//	{
//		auto hDelta = 1;
//		auto hPanning = hDelta * mouseSensitivity;
//
//		auto vDelta = 0;
//		auto vPanning = vDelta * mouseSensitivity;
//
//		auto viewMatrix = camera->GetViewMatrix();
//		MiniMath::V3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0), viewMatrix.at(2, 0));
//		MiniMath::V3 up(viewMatrix.at(0, 1), viewMatrix.at(1, 1), viewMatrix.at(2, 1));
//
//		auto target = camera->GetTarget();
//		auto eye = camera->GetEye();
//
//		if (hDelta != 0)
//		{
//			target -= right * (hPanning * radius * 0.01f);
//			eye -= right * (hPanning * radius * 0.01f);
//		}
//
//		if (vDelta != 0)
//		{
//			target += up * (vPanning * radius * 0.01f);
//			eye += up * (vPanning * radius * 0.01f);
//		}
//
//		camera->SetTarget(target);
//		camera->SetEye(eye);
//	}
//	else if (GLFW_KEY_RIGHT == event.keyEvent.keyCode)
//	{
//		auto hDelta = -1;
//		auto hPanning = hDelta * mouseSensitivity;
//
//		auto vDelta = 0;
//		auto vPanning = vDelta * mouseSensitivity;
//
//		auto viewMatrix = camera->GetViewMatrix();
//		MiniMath::V3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0), viewMatrix.at(2, 0));
//		MiniMath::V3 up(viewMatrix.at(0, 1), viewMatrix.at(1, 1), viewMatrix.at(2, 1));
//
//		auto target = camera->GetTarget();
//		auto eye = camera->GetEye();
//
//		if (hDelta != 0)
//		{
//			target -= right * (hPanning * radius * 0.01f);
//			eye -= right * (hPanning * radius * 0.01f);
//		}
//
//		if (vDelta != 0)
//		{
//			target += up * (vPanning * radius * 0.01f);
//			eye += up * (vPanning * radius * 0.01f);
//		}
//
//		camera->SetTarget(target);
//		camera->SetEye(eye);
//	}
//	else if (GLFW_KEY_L == event.keyEvent.keyCode)
//	{
//		camera->Reset();
//	}
//	else if (GLFW_KEY_INSERT == event.keyEvent.keyCode)
//	{
//		camera->PushCameraHistory();
//	}
//	else if (GLFW_KEY_DELETE == event.keyEvent.keyCode)
//	{
//		camera->PopCameraHistory();
//	}
//	else if (GLFW_KEY_PAGE_UP == event.keyEvent.keyCode)
//	{
//		camera->JumpToNextCameraHistory();
//	}
//	else if (GLFW_KEY_PAGE_DOWN == event.keyEvent.keyCode)
//	{
//		camera->JumpToPreviousCameraHistory();
//	}
//	else if (
//		GLFW_KEY_0 <= event.keyEvent.keyCode &&
//		GLFW_KEY_9 >= event.keyEvent.keyCode)
//	{
//		camera->JumpCameraHistory(event.keyEvent.keyCode - GLFW_KEY_0);
//	}
//}
//
//void CameraManipulatorOrbit::OnKeyRelease(const Event& event)
//{
//	pressedKeys.erase(event.keyEvent.keyCode);
//
//	if (nullptr == camera) return;
//}

CameraManipulatorTrackball::CameraManipulatorTrackball()
	: radius(10.0f),
	mouseSensitivity(0.005f),
	mousePanningSensitivity(0.01f),
	isLButtonPressed(false),
	isMButtonPressed(false),
	isRButtonPressed(false)
{
	//Feather.GetDispatcher().sink<KeyEvent>().connect<&CameraManipulatorTrackball::OnKey>(*this);
	//Feather.GetDispatcher().sink<MousePositionEvent>().connect<&CameraManipulatorTrackball::OnMousePosition>(*this);
	//Feather.GetDispatcher().sink<MouseButtonEvent>().connect<&CameraManipulatorTrackball::OnMouseButton>(*this);
	//Feather.GetDispatcher().sink<MouseWheelEvent>().connect<&CameraManipulatorTrackball::OnMouseWheel>(*this);
}

CameraManipulatorTrackball::~CameraManipulatorTrackball() {}

void CameraManipulatorTrackball::OnMousePosition(const MousePositionEvent& event)
{
	if (nullptr == camera) return;

	float dx = event.xpos - lastMousePositionX;
	float dy = event.ypos - lastMousePositionY;

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

	lastMousePositionX = event.xpos;
	lastMousePositionY = event.ypos;
}

void CameraManipulatorTrackball::OnMouseButton(const MouseButtonEvent& event)
{
	if (nullptr == camera) return;

	if (event.button == 0)
	{
		isLButtonPressed = event.action == 1;
	}
	else if (event.button == 1)
	{
		isRButtonPressed = event.action == 1;
	}
	else if (event.button == 2)
	{
		isMButtonPressed = event.action == 1;
	}

	lastMousePositionX = event.xpos;
	lastMousePositionY = event.ypos;
}

void CameraManipulatorTrackball::OnMouseWheel(const MouseWheelEvent& event)
{
	if (nullptr == camera) return;

	if (0 != pressedKeys.count(GLFW_KEY_LEFT_SHIFT) || 0 != pressedKeys.count(GLFW_KEY_RIGHT_SHIFT))
	{
		auto perspectiveCamera = dynamic_cast<PerspectiveCamera*>(camera);
		if (nullptr != perspectiveCamera)
		{
			f32 fovy = perspectiveCamera->GetFOVY() * RAD2DEG;

			if (0 > event.yoffset)
			{
				fovy += 1.0f;
			}
			else if (0 < event.yoffset)
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
		if (0 > event.yoffset)
		{
			radius *= 1.1f;
		}
		else if (0 < event.yoffset)
		{
			radius *= 0.9f;
		}

		auto eye = camera->GetEye();
		auto target = camera->GetTarget();

		MiniMath::V3 viewDir = MiniMath::normalize(eye - target);
		camera->SetEye(target + viewDir * radius);
	}
}

void CameraManipulatorTrackball::OnKey(const KeyEvent& event)
{
	pressedKeys.insert(event.keyCode);

	if (nullptr == camera) return;

	MiniMath::V3 eye = camera->GetEye();
	MiniMath::V3 target = camera->GetTarget();
	MiniMath::V3 up = camera->GetUp();
	MiniMath::V3 viewDir = MiniMath::normalize(target - eye);
	MiniMath::V3 right = MiniMath::normalize(MiniMath::cross(up, viewDir));

	float moveStep = 0.2f;

	if (event.keyCode == GLFW_KEY_W)
	{
		eye += viewDir * moveStep;
		target += viewDir * moveStep;
	}
	else if (event.keyCode == GLFW_KEY_S)
	{
		eye -= viewDir * moveStep;
		target -= viewDir * moveStep;
	}
	else if (event.keyCode == GLFW_KEY_A)
	{
		eye += right * moveStep;
		target += right * moveStep;
	}
	else if (event.keyCode == GLFW_KEY_D)
	{
		eye -= right * moveStep;
		target -= right * moveStep;
	}
	else if (event.keyCode == GLFW_KEY_R)
	{
		camera->Reset();
	}

	camera->SetEye(eye);
	camera->SetTarget(target);
}
