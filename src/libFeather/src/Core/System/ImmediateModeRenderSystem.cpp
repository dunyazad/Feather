#include <Core/System/ImmediateModeRenderSystem.h>
#include <Feather.h>
#include <Core/Component/Components.h>

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
	glViewport(0, 0, 3840, 2160);

	glLineWidth(2.0f);

	// Enable anti-aliasing for smoother lines
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Enable depth test if needed
	glEnable(GL_DEPTH_TEST);

	auto indices = Feather::GetInstance().GetComponentIDsByTypeIndex(typeid(PerspectiveCamera));
	for (auto& index : indices)
	{
		auto component = dynamic_cast<PerspectiveCamera*>(Feather::GetInstance().GetComponents()[index]);
		if (nullptr != component)
		{
			auto projection = component->GetProjectionMatrix();
			glMatrixMode(GL_PROJECTION);
			glLoadMatrixf((float*)projection.m);

			auto view = component->GetViewMatrix();
			glMatrixMode(GL_MODELVIEW); // ¸ðµ¨ºä Çà·Ä ¸ðµå·Î º¯°æ
			glLoadMatrixf((float*)view.m);
			
			//alog("%f, %f, %f, %f\n", m.at(0, 0), m.at(0, 1), m.at(0, 2), m.at(0, 3));
			//alog("%f, %f, %f, %f\n", m.at(1, 0), m.at(1, 1), m.at(1, 2), m.at(1, 3));
			//alog("%f, %f, %f, %f\n", m.at(2, 0), m.at(2, 1), m.at(2, 2), m.at(2, 3));
			//alog("%f, %f, %f, %f\n", m.at(3, 0), m.at(3, 1), m.at(3, 2), m.at(3, 3));
		}
	}

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
