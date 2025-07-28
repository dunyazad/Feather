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
		initialized = true;
	}
}

void VisualDebugging::Terminate()
{
	if (true == initialized)
	{
	}
}

void VisualDebugging::CreateEntity(const string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
	renderable->Initialize(Renderable::GeometryMode::Lines);

	entities[tag] = entity;
	debuggingRenderables[tag] = renderable;

	renderable->AddShader(Feather.CreateShader("Line", File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs")));
}

void VisualDebugging::Clear(const string& tag)
{
	if (false == initialized) Initialize();
	if (entities.end() == entities.find(tag)) CreateEntity(tag);

	auto& renderable = debuggingRenderables[tag]; 
	renderable->Clear();
}

void VisualDebugging::AddLine(const string& tag, const MiniMath::V3& v0, const MiniMath::V3& v1, const MiniMath::V4& c0, const MiniMath::V4& c1)
{
	if (false == initialized) Initialize();
	if (entities.end() == entities.find(tag)) CreateEntity(tag);

	auto& renderable = debuggingRenderables[tag];
	renderable->AddVertex(v0);
	renderable->AddVertex(v1);
	renderable->AddColor(c0);
	renderable->AddColor(c1);
}
