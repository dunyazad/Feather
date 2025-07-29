#include <VisualDebugging.h>

#include <Feather.h>
#include <GeometryBuilder.h>
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

void VisualDebugging::CreateLineEntity(const string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	entities[tag] = entity;

	auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
	renderable->Initialize(Renderable::GeometryMode::Lines);
	debuggingRenderables[tag] = renderable;

	renderable->AddShader(Feather.CreateShader("Line", File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs")));
}

void VisualDebugging::CreateTriangleEntity(const string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	entities[tag] = entity;

	auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
	renderable->Initialize(Renderable::GeometryMode::Triangles);
	debuggingRenderables[tag] = renderable;

	renderable->AddShader(Feather.CreateShader("Line", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
}

void VisualDebugging::CreateBoxEntity(const string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	entities[tag] = entity;

	auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
	renderable->Initialize(Renderable::GeometryMode::Triangles);
	debuggingRenderables[tag] = renderable;

	{
		auto shader = Feather.CreateShader("Instancing", File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
		renderable->AddShader(shader);
	}
	{
		auto shader = Feather.CreateShader("InstancingWithoutNormal", File("../../res/Shaders/InstancingWithoutNormal.vs"), File("../../res/Shaders/InstancingWithoutNormal.fs"));
		renderable->AddShader(shader);
	}
	renderable->SetActiveShaderIndex(1);

	auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildBox({ 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f });
	renderable->AddIndices(indices);
	renderable->AddVertices(vertices);
	renderable->AddNormals(normals);
	renderable->AddColors(colors);
	renderable->AddUVs(uvs);
}

void VisualDebugging::CreateSphereEntity(const string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	entities[tag] = entity;

	auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
	renderable->Initialize(Renderable::GeometryMode::Triangles);
	debuggingRenderables[tag] = renderable;

	{
		auto shader = Feather.CreateShader("Instancing", File("../../res/Shaders/Instancing.vs"), File("../../res/Shaders/Instancing.fs"));
		renderable->AddShader(shader);
	}
	{
		auto shader = Feather.CreateShader("InstancingWithoutNormal", File("../../res/Shaders/InstancingWithoutNormal.vs"), File("../../res/Shaders/InstancingWithoutNormal.fs"));
		renderable->AddShader(shader);
	}
	renderable->SetActiveShaderIndex(1);

	auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildSphere({ 0.0f, 0.0f, 0.0f }, 0.5f, 6, 6);
	renderable->AddIndices(indices);
	renderable->AddVertices(vertices);
	renderable->AddNormals(normals);
	renderable->AddColors(colors);
	renderable->AddUVs(uvs);
}

void VisualDebugging::Clear(const string& tag)
{
	if (false == initialized) Initialize();

	if (debuggingRenderables.end() != debuggingRenderables.find(tag))
	{
		auto& renderable = debuggingRenderables[tag];
		renderable->Clear();
	}
}

void VisualDebugging::ClearAll()
{
	for (auto& kvp : debuggingRenderables)
	{
		kvp.second->Clear();
	}
}

void VisualDebugging::AddLine(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c0, const glm::vec4& c1)
{
	if (false == initialized) Initialize();
	if (entities.end() == entities.find(tag)) CreateLineEntity(tag);

	auto& renderable = debuggingRenderables[tag];
	renderable->AddVertex(v0);
	renderable->AddVertex(v1);
	renderable->AddColor(c0);
	renderable->AddColor(c1);
}

void VisualDebugging::AddTriangle(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec4& c0, const glm::vec4& c1, const glm::vec4& c2)
{
	if (false == initialized) Initialize();
	if (entities.end() == entities.find(tag)) CreateTriangleEntity(tag);

	auto& renderable = debuggingRenderables[tag];
	auto i0 = renderable->AddVertex(v0);
	auto i1 = renderable->AddVertex(v1);
	auto i2 = renderable->AddVertex(v2);
	
	renderable->AddColor(c0);
	renderable->AddColor(c1);
	renderable->AddColor(c2);

	renderable->AddIndex(i0);
	renderable->AddIndex(i1);
	renderable->AddIndex(i2);
}

void VisualDebugging::AddBox(const string& tag, const glm::vec3& center, const glm::vec3& normal, const glm::vec3& dimensions, const glm::vec4& color)
{
	if (false == initialized) Initialize();
	if (entities.end() == entities.find(tag)) CreateBoxEntity(tag);

	auto& renderable = debuggingRenderables[tag];

	renderable->AddInstanceColor(color);
	renderable->AddInstanceNormal(normal);

	glm::mat4 tm = glm::identity<glm::mat4>();
	tm = glm::translate(tm, center);
	tm[0][0] = dimensions.x;
	tm[1][1] = dimensions.y;
	tm[2][2] = dimensions.z;
	renderable->AddInstanceTransform(tm);

	renderable->IncreaseNumberOfInstances();
}

void VisualDebugging::AddSphere(const string& tag, const glm::vec3& center, float radius, const glm::vec4& color)
{
	if (false == initialized) Initialize();
	if (entities.end() == entities.find(tag)) CreateSphereEntity(tag);

	auto& renderable = debuggingRenderables[tag];

	renderable->AddInstanceColor(color);
	renderable->AddInstanceNormal({0.0f, 0.1f, 0.0f});

	glm::mat4 tm = glm::translate(glm::mat4(1.0f), center) * glm::scale(glm::mat4(1.0f), glm::vec3(radius));
	renderable->AddInstanceTransform(tm);

	renderable->IncreaseNumberOfInstances();
}
