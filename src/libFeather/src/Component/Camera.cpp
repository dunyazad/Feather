#include <Component/Camera.h> 
#include <Feather.h>
#include <FeatherWindow.h>

Camera::Camera()
{
	auto window = Feather.GetFeatherWindow();
	if (window)
	{
		perspectiveSettings.SetAspectRatio((f32)window->GetWidth() / (f32)window->GetHeight());
	}

	mode = Perspective;
	dirty = true;
}

Camera::~Camera()
{
}

void Camera::SetProjectionMode(ProjectionMode newMode)
{
	if (mode != newMode)
	{
		mode = newMode;
		dirty = true;
	}
}

void Camera::Update(ui32 frameNo, f32 timeDelta)
{
	if(perspectiveSettings.IsDirty())
	{
		this->dirty = true;
	}

	if(orthogonalSettings.IsDirty())
	{
		this->dirty = true;
	}

	if (dirty)
	{
		if (mode == Perspective)
		{
			projectionMatrix = glm::perspective(
				perspectiveSettings.GetFovy(),
				perspectiveSettings.GetAspectRatio(),
				perspectiveSettings.GetZNear(),
				perspectiveSettings.GetZFar()
			);
		}
		else // Orthogonal
		{
			projectionMatrix = glm::ortho(
				orthogonalSettings.GetLeft(),
				orthogonalSettings.GetRight(),
				orthogonalSettings.GetBottom(),
				orthogonalSettings.GetTop(),
				orthogonalSettings.GetZNear(),
				orthogonalSettings.GetZFar()
			);
		}

		viewMatrix = glm::lookAt(eye, target, up);

		dirty = false;
	}
}

Ray Camera::ScreenPointToRay(float mouseX, float mouseY, int screenWidth, int screenHeight)
{
	// 행렬이 업데이트되지 않았을 경우를 대비해 필요하면 Update 호출 (선택 사항)
	// if (dirty) Update(0, 0.0f);

	// 1. NDC 좌표 계산 (-1 ~ 1)
	float x = (2.0f * mouseX) / (float)screenWidth - 1.0f;
	float y = 1.0f - (2.0f * mouseY) / (float)screenHeight;

	// 2. Ray의 시작점(Near)과 끝점(Far)을 NDC 상에서 정의
	glm::vec4 ray_begin = glm::vec4(x, y, -1.0f, 1.0f);
	glm::vec4 ray_end = glm::vec4(x, y, 1.0f, 1.0f);

	// 3. 역행렬을 이용해 World 좌표로 변환
	// (Perspective와 Orthogonal 모두 동일한 수식으로 동작합니다)
	auto inv = glm::inverse(projectionMatrix * viewMatrix);

	auto begin = inv * ray_begin;
	auto end = inv * ray_end;

	// 4. Perspective Divide (w로 나누기)
	begin /= begin.w;
	end /= end.w;

	// 5. Ray 구성
	glm::vec3 origin = glm::vec3(begin);
	glm::vec3 dir = glm::normalize(glm::vec3(end) - glm::vec3(begin));

	return { origin, dir };
}