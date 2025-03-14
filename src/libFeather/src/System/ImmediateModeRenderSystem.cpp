#include <System/ImmediateModeRenderSystem.h>
#include <Feather.h>
#include <FeatherWindow.h>
#include <Component/Components.h>

ImmediateModeRenderSystem::ImmediateModeRenderSystem(FeatherWindow* window)
	: RegisterDerivation<ImmediateModeRenderSystem, SystemBase>(window)
{
}

ImmediateModeRenderSystem::~ImmediateModeRenderSystem()
{
}

void ImmediateModeRenderSystem::Initialize()
{
}

void ImmediateModeRenderSystem::Terminate()
{
}

void ImmediateModeRenderSystem::Update(ui32 frameNo, f32 timeDelta)
{
	glUseProgram(0);

	glViewport(0, 0,
		Feather.GetFeatherWindow()->GetWidth(),
		Feather.GetFeatherWindow()->GetHeight());

	glPointSize(5.0f);
	glLineWidth(2.0f);

	// Enable anti-aliasing for smoother lines
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Enable depth test if needed
	glEnable(GL_DEPTH_TEST);

	auto cameras = Feather.GetInstances<PerspectiveCamera>();
	for (auto& camera : cameras)
	{
		if (nullptr != camera)
		{
			auto projection = camera->GetProjectionMatrix();
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf((float*)projection.m);

			auto view = camera->GetViewMatrix();
			auto target = camera->GetTarget();
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf((float*)view.m);

			glColor3f(1.0f, 0.0f, 0.0f);
			glBegin(GL_POINTS);
			glVertex3f(target.x, target.y, target.z);
			glEnd();
		}
	}

	// Draw X-axis (Red)
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-0.5f, 0.0f, 0.0f);
	glVertex3f(1.0f, 0.0f, 0.0f);
	glEnd();

	// Draw Y-axis (Green)
	glBegin(GL_LINES);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, -0.5f, 0.0f);
	glVertex3f(0.0f, 1.0f, 0.0f);
	glEnd();

	// Draw Z-axis (Blue)
	glBegin(GL_LINES);
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, -0.5f);
	glVertex3f(0.0f, 0.0f, 1.0f);
	glEnd();
}
