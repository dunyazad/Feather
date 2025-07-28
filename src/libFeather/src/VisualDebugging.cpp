#include <VisualDebugging.h>

#include <Feather.h>
#include <Component/Renderable.h>

bool VisualDebugging::initialized = false;
map<string, Entity> VisualDebugging::entities;
map<string, DebuggingRenderable*> VisualDebugging::debuggingRenderables;

void VisualDebugging::Initialize()
{
	if (false == initialized)
	{
		auto entity = Feather.CreateEntity("Lines");
		auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
		renderable->Initialize(Renderable::GeometryMode::Lines);

		entities["Lines"] = entity;
		debuggingRenderables["Lines"] = renderable;

		renderable->AddShader(Feather.CreateShader("Line", File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs")));
	}
}

void VisualDebugging::Terminate()
{
	if (true == initialized)
	{
	}
}

void VisualDebugging::AddLine(const string& tag, const MiniMath::V3& v0, const MiniMath::V3& v1, const MiniMath::V4& c0, const MiniMath::V4& c1)
{
	if (false == initialized)
	{
		Initialize();
	}

	auto& renderable = debuggingRenderables["Lines"];
	renderable->AddVertex(v0);
	renderable->AddVertex(v1);
	renderable->AddColor(c0);
	renderable->AddColor(c1);
}
