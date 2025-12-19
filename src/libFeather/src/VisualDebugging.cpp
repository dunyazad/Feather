#include <VisualDebugging.h>

#include <Feather.h>
#include <GeometryBuilder.h>
#include <Component/Renderable.h>
#include <Component/GUIComponent/GUIComponents.h>

bool VisualDebugging::initialized = false;
std::map<std::string, Entity> VisualDebugging::entities;
std::map<std::string, DebuggingRenderable*> VisualDebugging::debuggingRenderables;
std::map<std::string, TextBlock*> VisualDebugging::textBlocks;
std::vector<std::string> VisualDebugging::selectionRenderables;
size_t VisualDebugging::selectionIndex = 0;

std::mutex VisualDebugging::commandMutex;
std::vector<std::function<void()>> VisualDebugging::commandQueue;
std::vector<std::function<void()>> VisualDebugging::pendingCommands;

void VisualDebugging::Initialize()
{
	if (false == initialized)
	{
		initialized = true;
		pendingCommands.reserve(1000);
	}
}

void VisualDebugging::Terminate()
{
	if (true == initialized)
	{
		pendingCommands.clear();
	}
}

void VisualDebugging::DispatchCommands()
{
	{
		std::lock_guard<std::mutex> lock(commandMutex);
		if (!commandQueue.empty())
		{
			pendingCommands.insert(
				pendingCommands.end(),
				std::make_move_iterator(commandQueue.begin()),
				std::make_move_iterator(commandQueue.end())
			);
			commandQueue.clear();
		}
	}

	if (pendingCommands.empty()) return;

	constexpr long long kMaxExecutionTimeMicros = 2000; // 2ms
	auto startTime = std::chrono::high_resolution_clock::now();

	size_t processedCount = 0;
	for (const auto& command : pendingCommands)
	{
		command();
		processedCount++;

		auto currentTime = std::chrono::high_resolution_clock::now();
		auto elapsedMicros = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - startTime).count();

		if (elapsedMicros > kMaxExecutionTimeMicros)
		{
			break;
		}
	}

	if (processedCount > 0)
	{
		if (processedCount == pendingCommands.size())
		{
			pendingCommands.clear();
		}
		else
		{
			pendingCommands.erase(pendingCommands.begin(), pendingCommands.begin() + processedCount);
		}
	}
}

void VisualDebugging::CreateLineEntity(const std::string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	entities[tag] = entity;

	auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
	renderable->Initialize(Renderable::GeometryMode::Lines);
	debuggingRenderables[tag] = renderable;

	renderable->AddShader(Feather.CreateShader("Line", File("../../res/Shaders/Line.vs"), File("../../res/Shaders/Line.fs")));
}

void VisualDebugging::CreateTriangleEntity(const std::string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	entities[tag] = entity;

	auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
	renderable->Initialize(Renderable::GeometryMode::Triangles);
	debuggingRenderables[tag] = renderable;

	renderable->AddShader(Feather.CreateShader("Line", File("../../res/Shaders/Default.vs"), File("../../res/Shaders/Default.fs")));
}

void VisualDebugging::CreateBoxEntity(const std::string& tag)
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

void VisualDebugging::CreateWiredBoxEntity(const std::string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	entities[tag] = entity;

	auto renderable = Feather.CreateComponent<DebuggingRenderable>(entity);
	renderable->Initialize(Renderable::GeometryMode::Lines);
	debuggingRenderables[tag] = renderable;

	auto shader = Feather.CreateShader("InstancingWithoutLighting", File("../../res/Shaders/InstancingWithoutLighting.vs"), File("../../res/Shaders/InstancingWithoutLighting.fs"));
	renderable->AddShader(shader);

	auto [indices, vertices, normals, colors, uvs] = GeometryBuilder::BuildWiredBox({ 0.0f, 0.0f, 0.0f }, { 1.0f, 1.0f, 1.0f });
	renderable->AddIndices(indices);
	renderable->AddVertices(vertices);
	renderable->AddNormals(normals);
	renderable->AddColors(colors);
	renderable->AddUVs(uvs);
}

void VisualDebugging::CreateSphereEntity(const std::string& tag)
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

void VisualDebugging::CreateTextBlockEntity(const std::string& tag)
{
	auto entity = Feather.CreateEntity(tag);
	entities[tag] = entity;

	auto renderable = Feather.CreateComponent<TextBlock>(entity);
	textBlocks[tag] = renderable;
}

void VisualDebugging::Clear(const std::string& tag)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();

			if (debuggingRenderables.end() != debuggingRenderables.find(tag))
			{
				auto& renderable = debuggingRenderables[tag];
				if (renderable->IsInstancingEnabled())
				{
					renderable->ClearInstancingData();
				}
				else
				{
					renderable->Clear();
				}
				return;
			}

			if (textBlocks.end() != textBlocks.find(tag))
			{
				auto& textBlock = textBlocks[tag];
				textBlock->Clear();
				return;
			}
		});
}

void VisualDebugging::ClearAll()
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([]()
		{
			for (auto& kvp : debuggingRenderables)
			{
				if (kvp.second->IsInstancingEnabled())
				{
					kvp.second->ClearInstancingData();
				}
				else
				{
					kvp.second->Clear();
				}
			}

			for (auto& kvp : textBlocks)
			{
				kvp.second->Clear();
			}
		});
}

void VisualDebugging::SetVisibility(bool visible, const std::string& tag)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();

			if (debuggingRenderables.end() != debuggingRenderables.find(tag))
			{
				auto& renderable = debuggingRenderables[tag];
				renderable->SetVisible(visible);
				return;
			}

			if (textBlocks.end() != textBlocks.find(tag))
			{
				auto& textBlock = textBlocks[tag];
				textBlock->SetVisible(visible);
				return;
			}
		});
}

void VisualDebugging::SetVisibilityAll(bool visible)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();

			for (auto& kvp : debuggingRenderables)
			{
				kvp.second->SetVisible(visible);
			}

			for (auto& kvp : textBlocks)
			{
				kvp.second->SetVisible(visible);
			}
		});
}

void VisualDebugging::ToggleVisibility(const std::string& tag)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();

			if (debuggingRenderables.end() != debuggingRenderables.find(tag))
			{
				auto& renderable = debuggingRenderables[tag];
				renderable->SetVisible(!renderable->IsVisible());
				return;
			}
			if (textBlocks.end() != textBlocks.find(tag))
			{
				auto& textBlock = textBlocks[tag];
				textBlock->SetVisible(!textBlock->IsVisible());
				return;
			}
		});
}

void VisualDebugging::ToggleVisibilityAll()
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([]()
		{
			if (false == initialized) Initialize();

			for (auto& kvp : debuggingRenderables)
			{
				kvp.second->SetVisible(!kvp.second->IsVisible());
			}
			for (auto& kvp : textBlocks)
			{
				kvp.second->SetVisible(!kvp.second->IsVisible());
			}
		});
}

void VisualDebugging::AddLine(const std::string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c)
{
	AddLine(tag, v0, v1, c, c);
}

void VisualDebugging::AddLine(const std::string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c0, const glm::vec4& c1)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();
			if (entities.end() == entities.find(tag)) CreateLineEntity(tag);

			auto& renderable = debuggingRenderables[tag];
			renderable->AddVertex(v0);
			renderable->AddVertex(v1);
			renderable->AddColor(c0);
			renderable->AddColor(c1);
		});
}

void VisualDebugging::AddLine(const std::string& tag, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector4f& c)
{
	AddLine(tag,
		glm::vec3(v0.x(), v0.y(), v0.z()),
		glm::vec3(v1.x(), v1.y(), v1.z()),
		glm::vec4(c.x(), c.y(), c.z(), c.w()),
		glm::vec4(c.x(), c.y(), c.z(), c.w()));
}

void VisualDebugging::AddLine(const std::string& tag, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector4f& c0, const Eigen::Vector4f& c1)
{
	AddLine(tag,
		glm::vec3(v0.x(), v0.y(), v0.z()),
		glm::vec3(v1.x(), v1.y(), v1.z()),
		glm::vec4(c0.x(), c0.y(), c0.z(), c0.w()),
		glm::vec4(c1.x(), c1.y(), c1.z(), c1.w()));
}

void VisualDebugging::AddTriangle(const std::string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec4& c)
{
	AddTriangle(tag, v0, v1, v2, c, c, c);
}

void VisualDebugging::AddTriangle(const std::string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec4& c0, const glm::vec4& c1, const glm::vec4& c2)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();
			if (entities.end() == entities.find(tag)) CreateTriangleEntity(tag);

			auto& renderable = debuggingRenderables[tag];
			auto i0 = renderable->AddVertex(v0);
			auto i1 = renderable->AddVertex(v1);
			auto i2 = renderable->AddVertex(v2);

			auto normal = glm::trianglenormal(v0, v1, v2);

			renderable->AddNormal(normal);
			renderable->AddNormal(normal);
			renderable->AddNormal(normal);

			renderable->AddColor(c0);
			renderable->AddColor(c1);
			renderable->AddColor(c2);

			renderable->AddIndex(i0);
			renderable->AddIndex(i1);
			renderable->AddIndex(i2);
		});
}

void VisualDebugging::AddTriangle(const std::string& tag, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector4f& c)
{
	AddTriangle(tag, v0, v1, v2, c, c, c);
}

void VisualDebugging::AddTriangle(const std::string& tag, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector4f& c0, const Eigen::Vector4f& c1, const Eigen::Vector4f& c2)
{
	AddTriangle(tag,
		glm::vec3(v0.x(), v0.y(), v0.z()),
		glm::vec3(v1.x(), v1.y(), v1.z()),
		glm::vec3(v2.x(), v2.y(), v2.z()),
		glm::vec4(c0.x(), c0.y(), c0.z(), c0.w()),
		glm::vec4(c1.x(), c1.y(), c1.z(), c1.w()),
		glm::vec4(c2.x(), c2.y(), c2.z(), c2.w()));
}

void VisualDebugging::AddBox(const std::string& tag, const AABB& aabb, const glm::vec4& color)
{
	auto center = (aabb.min + aabb.max) * 0.5f;
	auto dimensions = aabb.max - aabb.min;
	AddBox(tag, center, { 0.0f, 0.0f, 1.0f }, dimensions, color);
}

void VisualDebugging::AddBox(const std::string& tag, const Eigen::AABB& aabb, const Eigen::Vector4f& color)
{
	auto center = (aabb.min + aabb.max) * 0.5f;
	auto dimensions = aabb.max - aabb.min;
	AddBox(tag, center, { 0.0f, 0.0f, 1.0f }, dimensions, color);
}

void VisualDebugging::AddBox(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& normal, const Eigen::Vector3f& dimensions, const Eigen::Vector4f& color)
{
	AddBox(tag,
		glm::vec3(center.x(), center.y(), center.z()),
		glm::vec3(normal.x(), normal.y(), normal.z()),
		glm::vec3(dimensions.x(), dimensions.y(), dimensions.z()),
		glm::vec4(color.x(), color.y(), color.z(), color.w()));
}

void VisualDebugging::AddBox(const std::string& tag, const glm::vec3& center, const glm::vec3& dimensions, const glm::vec4& color)
{
	AddBox(tag, center, glm::vec3(0.0f, 1.0f, 0.0f), dimensions, color);
}

void VisualDebugging::AddBox(const std::string& tag, const glm::vec3& center, const glm::vec3& normal, const glm::vec3& dimensions, const glm::vec4& color)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();
			if (entities.end() == entities.find(tag)) CreateBoxEntity(tag);

			auto& renderable = debuggingRenderables[tag];

			renderable->AddInstanceColor(color);
			renderable->AddInstanceNormal(normal);

			glm::mat4 tm = glm::identity<glm::mat4>();
			glm::mat4 rot = glm::mat4(1.0f);
			if (glm::length(normal) > 0.0001f)
			{
				glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(0, 0, 1), normal));
				float angle = acos(glm::dot(glm::normalize(normal), glm::vec3(0, 0, 1)));
				if (glm::length(axis) > 0.0001f)
					rot = glm::rotate(glm::mat4(1.0f), angle, axis);
			}
			tm = glm::translate(tm, center) * rot * glm::scale(glm::mat4(1.0f), dimensions);
			renderable->AddInstanceTransform(tm);

			renderable->IncreaseNumberOfInstances();
		});
}

void VisualDebugging::AddBox(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& dimensions, const Eigen::Vector4f& color)
{
	AddBox(tag,
		glm::vec3(center.x(), center.y(), center.z()),
		glm::vec3(dimensions.x(), dimensions.y(), dimensions.z()),
		glm::vec4(color.x(), color.y(), color.z(), color.w()));
}

void VisualDebugging::AddWiredBox(const std::string& tag, const AABB& aabb, const glm::vec4& color)
{
	auto center = (aabb.min + aabb.max) * 0.5f;
	auto dimensions = aabb.max - aabb.min;
	AddWiredBox(tag, center, { 0.0f, 0.0f, 1.0f }, dimensions, color);
}

void VisualDebugging::AddWiredBox(const std::string& tag, const glm::vec3& center, const glm::vec3& dimensions, const glm::vec4& color)
{
	AddWiredBox(tag, center, glm::vec3(0.0f, 1.0f, 0.0f), dimensions, color);
}

void VisualDebugging::AddWiredBox(const std::string& tag, const glm::vec3& center, const glm::vec3& normal, const glm::vec3& dimensions, const glm::vec4& color)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();
			if (entities.end() == entities.find(tag)) CreateWiredBoxEntity(tag);

			auto& renderable = debuggingRenderables[tag];

			renderable->AddInstanceColor(color);
			renderable->AddInstanceNormal(normal);

			glm::mat4 tm = glm::identity<glm::mat4>();
			glm::mat4 rot = glm::mat4(1.0f);
			if (glm::length(normal) > 0.0001f)
			{
				glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(0, 0, 1), normal));
				float angle = acos(glm::dot(glm::normalize(normal), glm::vec3(0, 0, 1)));
				if (glm::length(axis) > 0.0001f)
					rot = glm::rotate(glm::mat4(1.0f), angle, axis);
			}
			tm = glm::translate(tm, center) * rot * glm::scale(glm::mat4(1.0f), dimensions);
			renderable->AddInstanceTransform(tm);

			renderable->IncreaseNumberOfInstances();
		});
}

void VisualDebugging::AddWiredBox(const std::string& tag, const Eigen::AABB& aabb, const Eigen::Vector4f& color)
{
	AddWiredBox(tag, { glm::vec3(aabb.min.x(), aabb.min.y(), aabb.min.z()) }, { glm::vec3(aabb.max.x(), aabb.max.y(), aabb.max.z()) }, glm::vec4(color.x(), color.y(), color.z(), color.w()));
}

void VisualDebugging::AddWiredBox(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& dimensions, const Eigen::Vector4f& color)
{
	AddWiredBox(tag,
		glm::vec3(center.x(), center.y(), center.z()),
		glm::vec3(dimensions.x(), dimensions.y(), dimensions.z()),
		glm::vec4(color.x(), color.y(), color.z(), color.w()));
}

void VisualDebugging::AddWiredBox(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& normal, const Eigen::Vector3f& dimensions, const Eigen::Vector4f& color)
{
	AddWiredBox(tag,
		glm::vec3(center.x(), center.y(), center.z()),
		glm::vec3(normal.x(), normal.y(), normal.z()),
		glm::vec3(dimensions.x(), dimensions.y(), dimensions.z()),
		glm::vec4(color.x(), color.y(), color.z(), color.w()));
}

void VisualDebugging::AddSphere(const std::string& tag, const glm::vec3& center, float radius, const glm::vec4& color)
{
	AddSphere(tag, center, glm::vec3(0.0f, 1.0f, 0.0f), radius, color);
}

void VisualDebugging::AddSphere(const std::string& tag, const glm::vec3& center, const glm::vec3& normal, float radius, const glm::vec4& color)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();
			if (entities.end() == entities.find(tag)) CreateSphereEntity(tag);

			auto& renderable = debuggingRenderables[tag];

			renderable->AddInstanceColor(color);
			renderable->AddInstanceNormal({ 0.0f, 0.1f, 0.0f });

			glm::mat4 tm = glm::identity<glm::mat4>();
			glm::mat4 rot = glm::mat4(1.0f);
			if (glm::length(normal) > 0.0001f)
			{
				glm::vec3 axis = glm::normalize(glm::cross(glm::vec3(0, 0, 1), normal));
				float angle = acos(glm::dot(glm::normalize(normal), glm::vec3(0, 0, 1)));
				if (glm::length(axis) > 0.0001f)
					rot = glm::rotate(glm::mat4(1.0f), angle, axis);
			}
			tm = glm::translate(tm, center) * rot * glm::scale(glm::mat4(1.0f), glm::vec3(radius * 2.0f));
			renderable->AddInstanceTransform(tm);

			renderable->IncreaseNumberOfInstances();
		});
}

void VisualDebugging::AddSphere(const std::string& tag, const Eigen::Vector3f& center, float radius, const Eigen::Vector4f& color)
{
	AddSphere(tag,
		glm::vec3(center.x(), center.y(), center.z()),
		radius,
		glm::vec4(color.x(), color.y(), color.z(), color.w()));
}

void VisualDebugging::AddSphere(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& normal, float radius, const Eigen::Vector4f& color)
{
	AddSphere(tag,
		glm::vec3(center.x(), center.y(), center.z()),
		glm::vec3(normal.x(), normal.y(), normal.z()),
		radius,
		glm::vec4(color.x(), color.y(), color.z(), color.w()));
}

void VisualDebugging::AddText(const std::string& tag, const std::string& text, const glm::vec3& position, const glm::vec4& color, float fontSize)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();
			if (entities.end() == entities.find(tag)) CreateTextBlockEntity(tag);

			auto& textBlock = textBlocks[tag];
			textBlock->AddText(text, position, color, fontSize);
		});
}

void VisualDebugging::AddText(const std::string& tag, const std::string& text, const Eigen::Vector3f& position, const Eigen::Vector4f& color, float fontSize)
{
	AddText(tag, text,
		glm::vec3(position.x(), position.y(), position.z()),
		glm::vec4(color.x(), color.y(), color.z(), color.w()),
		fontSize);
}

void VisualDebugging::ClearSelectionList()
{
	selectionRenderables.clear();
	selectionIndex = 0;
}

void VisualDebugging::AddToSelectionList(const std::string& tag)
{
	std::lock_guard<std::mutex> lock(commandMutex);
	commandQueue.emplace_back([=]()
		{
			if (false == initialized) Initialize();

			if (debuggingRenderables.end() != debuggingRenderables.find(tag))
			{
				//if (std::find(selectionRenderables.begin(), selectionRenderables.end(), tag) == selectionRenderables.end())

				selectionRenderables.push_back(tag);
			}
			else
			{
				printf("Warning: Tag not found %s\n", tag.c_str());
			}
		});
}

unsigned int VisualDebugging::ShowNextSelection()
{
	for (auto& tag : selectionRenderables)
	{
		SetVisibility(false, tag);
	}

	selectionIndex++;
	selectionIndex = selectionIndex % selectionRenderables.size();

	auto& tag = selectionRenderables[selectionIndex];
	SetVisibility(true, tag);

	return selectionIndex;
}

unsigned int VisualDebugging::ShowPreviousSelection()
{
	for (auto& tag : selectionRenderables)
	{
		SetVisibility(false, tag);
	}

	if (0 == selectionIndex) selectionIndex += selectionRenderables.size();
	selectionIndex--;

	auto& tag = selectionRenderables[selectionIndex];
	SetVisibility(true, tag);

	return selectionIndex;
}
