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

	glPointSize(10.0f);
	glLineWidth(2.0f);

	glClear(GL_DEPTH_BUFFER_BIT);

	// Enable anti-aliasing for smoother lines
	//glEnable(GL_LINE_SMOOTH);
	//glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Enable depth test if needed
	glEnable(GL_DEPTH_TEST);

	// Draw X-axis (Red)
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-100.f, 0.0f, 0.0f);
	glVertex3f(100.0f, 0.0f, 0.0f);
	glEnd();

	// Draw Y-axis (Green)
	glBegin(GL_LINES);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, -100.0f, 0.0f);
	glVertex3f(0.0f, 100.0f, 0.0f);
	glEnd();

	// Draw Z-axis (Blue)
	glBegin(GL_LINES);
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, -100.0f);
	glVertex3f(0.0f, 0.0f, 100.0f);
	glEnd();

	auto cameras = Feather.GetInstances<PerspectiveCamera>();
	for (auto& camera : cameras)
	{
		if (nullptr != camera)
		{
			const auto& projection = camera->GetProjectionMatrix();
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf((float*)projection.m);

			const auto& view = camera->GetViewMatrix();
			auto target = camera->GetTarget();
			glMatrixMode(GL_MODELVIEW);
			glLoadMatrixf((float*)view.m);

			glColor3f(1.0f, 0.0f, 0.0f);
			glBegin(GL_POINTS);
			glVertex3f(target.x, target.y, target.z);
			glEnd();

			// Draw X-axis (Red)
			glBegin(GL_LINES);
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f(target.x - 0.5f, target.y, target.z);
			glVertex3f(target.x + 1.0f, target.y, target.z);
			glEnd();

			// Draw Y-axis (Green)
			glBegin(GL_LINES);
			glColor3f(0.0f, 1.0f, 0.0f);
			glVertex3f(target.x, target.y - 0.5f, target.z);
			glVertex3f(target.x, target.y + 1.0f, target.z);
			glEnd();

			// Draw Z-axis (Blue)
			glBegin(GL_LINES);
			glColor3f(0.0f, 0.0f, 1.0f);
			glVertex3f(target.x, target.y, target.z - 0.5f);
			glVertex3f(target.x, target.y, target.z + 1.0f);
			glEnd();
		}
	}
}