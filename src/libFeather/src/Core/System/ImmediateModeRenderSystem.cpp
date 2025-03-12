#include <Core/System/ImmediateModeRenderSystem.h>

ImmediateModeRenderSystem::ImmediateModeRenderSystem(FeatherWindow* window)
	: SystemBase(window)
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
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	//glBegin(GL_LINES);
	//glColor3f(1.0f, 0.0f, 0.0f);
	//glVertex3f(-0.5f, 0.0f, 0.0f);
	//glVertex3f(0.5f, 0.0f, 0.0f);

	//glColor3f(0.0f, 1.0f, 0.0f);
	//glVertex3f(0.0f, -0.5f, 0.0f);
	//glVertex3f(0.0f, 0.5f, 0.0f);

	//glColor3f(0.0f, 0.0f, 1.0f);
	//glVertex3f(0.0f, 0.0f, -0.5f);
	//glVertex3f(0.0f, 0.0f, 0.5f);
	//glEnd();




   // Set up coordinate system
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-150.0, 150.0, -150.0, 150.0, -150.0, 150.0); // Adjusted for better visibility

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	// Set line width
	glLineWidth(2.0f);

	// Enable anti-aliasing for smoother lines
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Enable depth test if needed
	glEnable(GL_DEPTH_TEST);

	// Draw X-axis (Red)
	glBegin(GL_LINES);
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(-100.0f, 0.0f, 0.0f);
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

}
