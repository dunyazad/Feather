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
//		glm::vec3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0), viewMatrix.at(2, 0));
//		glm::vec3 up(viewMatrix.at(0, 1), viewMatrix.at(1, 1), viewMatrix.at(2, 1));
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
//		glm::vec3 right(viewMatrix.at(0, 0), viewMatrix.at(1, 0), viewMatrix.at(2, 0));
//		glm::vec3 up(viewMatrix.at(0, 1), viewMatrix.at(1, 1), viewMatrix.at(2, 1));
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
	: radius(50.0f),
	mouseSensitivity(0.005f),
	mousePanningSensitivity(0.01f),
	isLButtonPressed(false),
	isMButtonPressed(false),
	isRButtonPressed(false)
{
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
		float angleY = -dy * mouseSensitivity;

		glm::vec3 eye = camera->GetEye();
		glm::vec3 target = camera->GetTarget();
		glm::vec3 up = camera->GetUp();

		glm::vec3 viewDir = glm::normalize(eye - target);
		glm::vec3 right = glm::normalize(glm::cross(up, viewDir));

		glm::quat rotX = glm::angleAxis(angleX, up);
		glm::quat rotY = glm::angleAxis(angleY, right);

		glm::quat rot = rotX * rotY;

		glm::vec3 rotatedViewDir = glm::normalize(rot * viewDir);
		glm::vec3 rotatedUp = glm::normalize(rot * up);

		glm::vec3 newEye = target + rotatedViewDir * radius;

		camera->SetEye(newEye);
		camera->SetUp(rotatedUp);
	}

	if (isMButtonPressed)
	{
		float panX = -dx * mousePanningSensitivity;
		float panY = dy * mousePanningSensitivity;

		glm::mat4 invView = glm::inverse(camera->GetViewMatrix());
		glm::vec3 screenRight = glm::normalize(glm::vec3(invView * glm::vec4(1, 0, 0, 0)));
		glm::vec3 screenUp = glm::normalize(glm::vec3(invView * glm::vec4(0, 1, 0, 0)));

		glm::vec3 eye = camera->GetEye();
		glm::vec3 target = camera->GetTarget();

		glm::vec3 offset = screenRight * panX + screenUp * panY;
		camera->SetEye(eye + offset);
		camera->SetTarget(target + offset);
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

	auto perspectiveCamera = dynamic_cast<PerspectiveCamera*>(camera);

	if (0 != pressedKeys.count(GLFW_KEY_LEFT_SHIFT) || 0 != pressedKeys.count(GLFW_KEY_RIGHT_SHIFT))
	{
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

		if (radius < perspectiveCamera->GetNear())
		{
			radius = perspectiveCamera->GetNear();
		}

		if (radius > perspectiveCamera->GetFar())
		{
			radius = perspectiveCamera->GetFar();
		}

		auto eye = camera->GetEye();
		auto target = camera->GetTarget();

		glm::vec3 viewDir = glm::normalize(eye - target);
		camera->SetEye(target + viewDir * radius);
	}
}

void CameraManipulatorTrackball::OnKey(const KeyEvent& event)
{
	pressedKeys.insert(event.keyCode);

	if (nullptr == camera) return;

	glm::vec3 eye = camera->GetEye();
	glm::vec3 target = camera->GetTarget();
	glm::vec3 up = camera->GetUp();
	glm::vec3 viewDir = glm::normalize(target - eye);
	glm::vec3 right = glm::normalize(glm::cross(up, viewDir));

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
		Reset();
		return;
	}

	camera->SetEye(eye);
	camera->SetTarget(target);
}

void CameraManipulatorTrackball::PushCameraHistory()
{
	glm::vec3 eye = camera->GetEye();
	glm::vec3 target = camera->GetTarget();
	glm::vec3 up = camera->GetUp();

	cameraHistory.push_back({ eye, target, up, radius });

	JumpCameraHistory(cameraHistory.size() - 1);
}

void CameraManipulatorTrackball::PopCameraHistory()
{
	if (1 >= cameraHistory.size()) return;

	cameraHistory.pop_back();

	if (cameraHistoryIndex >= cameraHistory.size())
	{
		JumpCameraHistory(cameraHistory.size() - 1);
	}
}

void CameraManipulatorTrackball::JumpCameraHistory(i32 index)
{
	if (index >= cameraHistory.size()) return;

	cameraHistoryIndex = index;

	auto [eye, target, up, radius] = cameraHistory[cameraHistoryIndex];

	camera->SetEye(eye);
	camera->SetTarget(target);
	camera->SetUp(up);
}

void CameraManipulatorTrackball::JumpToPreviousCameraHistory()
{
	if (0 == cameraHistoryIndex) return;

	JumpCameraHistory(cameraHistoryIndex - 1);
}

void CameraManipulatorTrackball::JumpToNextCameraHistory()
{
	if (cameraHistoryIndex >= cameraHistory.size()) return;

	JumpCameraHistory(cameraHistoryIndex + 1);
}

void CameraManipulatorTrackball::Reset()
{
	auto [eye, target, up, radius] = cameraHistory.front();
	this->radius = radius;

	cameraHistory.clear();
	cameraHistory.push_back({ eye, target, up, radius });

	cameraHistoryIndex = 0;

	camera->SetEye(eye);
	camera->SetTarget(target);
	camera->SetUp(up);
}

void CameraManipulatorTrackball::MakeDefault()
{
	glm::vec3 eye = camera->GetEye();
	glm::vec3 target = camera->GetTarget();
	glm::vec3 up = camera->GetUp();

	cameraHistory.clear();
	cameraHistory.push_back({ eye, target, up, radius });

	cameraHistoryIndex = 0;
}