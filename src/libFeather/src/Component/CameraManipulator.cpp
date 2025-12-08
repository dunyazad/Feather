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
		// Orbit Rotation
		float angleX = -dx * mouseSensitivity;
		float angleY = -dy * mouseSensitivity;

		glm::vec3 eye = camera->GetEye();
		glm::vec3 target = camera->GetTarget();
		glm::vec3 up = camera->GetUp();

		glm::vec3 viewDir = glm::normalize(eye - target);
		glm::vec3 right = glm::normalize(glm::cross(up, viewDir));

		// Rotate around Up vector (Yaw) and Right vector (Pitch)
		glm::quat rotX = glm::angleAxis(angleX, up); // World Up or Local Up logic can vary
		glm::quat rotY = glm::angleAxis(angleY, right);

		glm::quat rot = rotX * rotY;

		glm::vec3 rotatedViewDir = glm::normalize(rot * viewDir);
		glm::vec3 rotatedUp = glm::normalize(rot * up);

		// Re-calculate eye position based on target and radius
		// Note: We use the current distance to maintain stability if radius drift occurs
		float currentDist = glm::length(eye - target);
		glm::vec3 newEye = target + rotatedViewDir * currentDist;

		camera->SetEye(newEye);
		camera->SetUp(rotatedUp);
	}

	if (isMButtonPressed)
	{
		// Panning
		float panX = -dx * mousePanningSensitivity;
		float panY = dy * mousePanningSensitivity;

		glm::mat4 invView = glm::inverse(camera->GetViewMatrix());
		glm::vec3 screenRight = glm::normalize(glm::vec3(invView * glm::vec4(1, 0, 0, 0)));
		glm::vec3 screenUp = glm::normalize(glm::vec3(invView * glm::vec4(0, 1, 0, 0)));

		glm::vec3 eye = camera->GetEye();
		glm::vec3 target = camera->GetTarget();

		// Scale panning speed by radius for natural feel
		glm::vec3 offset = screenRight * panX * radius * mouseSensitivity * 10.0f +
			screenUp * panY * radius * mouseSensitivity * 10.0f;

		camera->SetEye(eye + offset);
		camera->SetTarget(target + offset);
	}

	lastMousePositionX = event.xpos;
	lastMousePositionY = event.ypos;
}

void CameraManipulatorTrackball::OnMouseButton(const MouseButtonEvent& event)
{
	if (nullptr == camera) return;

	// 0: Left, 1: Right, 2: Middle (GLFW Standard usually)
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

	bool isShiftPressed = (0 != pressedKeys.count(GLFW_KEY_LEFT_SHIFT) || 0 != pressedKeys.count(GLFW_KEY_RIGHT_SHIFT));

	// 1. Perspective Zoom (Radius or FOV)
	if (camera->GetProjectionMode() == Camera::Perspective)
	{
		if (isShiftPressed)
		{
			// Change FOV
			auto& settings = camera->GetPerspectiveSettings();
			f32 fovyDeg = settings.GetFovy() * RAD2DEG;

			if (event.yoffset < 0) fovyDeg += 1.0f;
			else if (event.yoffset > 0) fovyDeg -= 1.0f;

			if (fovyDeg < 1.0f) fovyDeg = 1.0f;
			if (fovyDeg > 179.0f) fovyDeg = 179.0f;

			settings.SetFovy(fovyDeg * DEG2RAD);
			camera->SetDirty(true);
		}
		else
		{
			// Change Distance (Dolly)
			if (event.yoffset < 0) radius *= 1.1f;
			else if (event.yoffset > 0) radius *= 0.9f;

			auto& settings = camera->GetPerspectiveSettings();
			if (radius < settings.GetZNear()) radius = settings.GetZNear();
			if (radius > settings.GetZFar()) radius = settings.GetZFar();

			auto eye = camera->GetEye();
			auto target = camera->GetTarget();

			glm::vec3 viewDir = glm::normalize(eye - target);
			camera->SetEye(target + viewDir * radius);
		}
	}
	// 2. Orthogonal Zoom (Scale View Volume)
	else if (camera->GetProjectionMode() == Camera::Orthogonal)
	{
		// For Ortho, moving the camera (radius) doesn't zoom. We must scale bounds.
		float scaleFactor = (event.yoffset > 0) ? 0.9f : 1.1f;

		auto& ortho = camera->GetOrthogonalSettings();

		ortho.SetTop(ortho.GetTop() * scaleFactor);
		ortho.SetBottom(ortho.GetBottom() * scaleFactor);
		ortho.SetLeft(ortho.GetLeft() * scaleFactor);
		ortho.SetRight(ortho.GetRight() * scaleFactor);

		// Optional: Clamp minimum zoom to prevent flipping or zero
		float minSize = 0.01f;
		if (abs(ortho.GetRight() - ortho.GetLeft()) < minSize)
		{
			// Prevent getting too small
			// Logic to reset to minSize could go here if needed
		}

		camera->SetDirty(true);
	}
}

void CameraManipulatorTrackball::OnKey(const KeyEvent& event)
{
	if (event.action == 1) // Press
	{
		pressedKeys.insert(event.keyCode);
	}
	else if (event.action == 0) // Release
	{
		pressedKeys.erase(event.keyCode);
	}

	if (nullptr == camera) return;

	glm::vec3 eye = camera->GetEye();
	glm::vec3 target = camera->GetTarget();
	glm::vec3 up = camera->GetUp();
	glm::vec3 viewDir = glm::normalize(target - eye);
	glm::vec3 right = glm::normalize(glm::cross(up, viewDir));

	float moveStep = 0.2f;

	// Simple WASD Fly controls (Moves both Eye and Target)
	if (event.keyCode == GLFW_KEY_W && event.action != 0)
	{
		eye += viewDir * moveStep;
		target += viewDir * moveStep;
	}
	else if (event.keyCode == GLFW_KEY_S && event.action != 0)
	{
		eye -= viewDir * moveStep;
		target -= viewDir * moveStep;
	}
	else if (event.keyCode == GLFW_KEY_A && event.action != 0)
	{
		eye -= right * moveStep;
		target -= right * moveStep;
	}
	else if (event.keyCode == GLFW_KEY_D && event.action != 0)
	{
		eye += right * moveStep;
		target += right * moveStep;
	}
	else if (event.keyCode == GLFW_KEY_R && event.action == 1)
	{
		Reset();
		return;
	}
	else if (event.keyCode == GLFW_KEY_P && event.action == 1)
	{
		// Switch Mode
		if (camera->GetProjectionMode() == Camera::Perspective)
		{
			// When switching to Ortho, set reasonable bounds based on current distance
			float dist = glm::length(target - eye);
			auto window = Feather.GetFeatherWindow();
			float aspect = (float)window->GetWidth() / (float)window->GetHeight();

			// Approximate bounds to match current perspective view at target distance
			// height ~ 2 * dist * tan(fovy/2)
			float fovy = camera->GetPerspectiveSettings().GetFovy();
			float height = 2.0f * dist * tan(fovy * 0.5f);
			float width = height * aspect;

			auto& ortho = camera->GetOrthogonalSettings();
			ortho.SetTop(height * 0.5f);
			ortho.SetBottom(-height * 0.5f);
			ortho.SetRight(width * 0.5f);
			ortho.SetLeft(-width * 0.5f);

			camera->SetProjectionMode(Camera::Orthogonal);
		}
		else
		{
			camera->SetProjectionMode(Camera::Perspective);
		}
		return;
	}

	camera->SetEye(eye);
	camera->SetTarget(target);
}

void CameraManipulatorTrackball::PushCameraHistory()
{
	if (!camera) return;

	glm::vec3 eye = camera->GetEye();
	glm::vec3 target = camera->GetTarget();
	glm::vec3 up = camera->GetUp();

	cameraHistory.push_back({ eye, target, up, radius });

	JumpCameraHistory((i32)cameraHistory.size() - 1);
}

void CameraManipulatorTrackball::PopCameraHistory()
{
	if (cameraHistory.size() <= 1) return;

	cameraHistory.pop_back();

	if (cameraHistoryIndex >= cameraHistory.size())
	{
		JumpCameraHistory((i32)cameraHistory.size() - 1);
	}
}

void CameraManipulatorTrackball::JumpCameraHistory(i32 index)
{
	if (index < 0 || index >= (i32)cameraHistory.size() || !camera) return;

	cameraHistoryIndex = index;

	auto [eye, target, up, savedRadius] = cameraHistory[cameraHistoryIndex];
	this->radius = savedRadius;

	camera->SetEye(eye);
	camera->SetTarget(target);
	camera->SetUp(up);
}

void CameraManipulatorTrackball::JumpToPreviousCameraHistory()
{
	if (cameraHistoryIndex == 0) return;

	JumpCameraHistory((i32)cameraHistoryIndex - 1);
}

void CameraManipulatorTrackball::JumpToNextCameraHistory()
{
	if (cameraHistoryIndex >= cameraHistory.size() - 1) return;

	JumpCameraHistory((i32)cameraHistoryIndex + 1);
}

void CameraManipulatorTrackball::Reset()
{
	if (cameraHistory.empty() || !camera) return;

	auto [eye, target, up, savedRadius] = cameraHistory.front();
	this->radius = savedRadius;

	// Reset logic: clear history but keep initial
	cameraHistory.clear();
	cameraHistory.push_back({ eye, target, up, savedRadius });

	cameraHistoryIndex = 0;

	camera->SetEye(eye);
	camera->SetTarget(target);
	camera->SetUp(up);

	// Reset mode to default if desired
	camera->SetProjectionMode(Camera::Perspective);
}

void CameraManipulatorTrackball::MakeDefault()
{
	if (!camera) return;

	glm::vec3 eye = camera->GetEye();
	glm::vec3 target = camera->GetTarget();
	glm::vec3 up = camera->GetUp();

	cameraHistory.clear();
	cameraHistory.push_back({ eye, target, up, radius });

	cameraHistoryIndex = 0;
}