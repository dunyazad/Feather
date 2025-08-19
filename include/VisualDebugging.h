#pragma once

#include <FeatherCommon.h>

class DebuggingRenderable;
class TextBlock;

class VisualDebugging
{
public:
	static VisualDebugging& Instance()
	{
		static VisualDebugging instance;
		return instance;
	}

	static void Initialize();
	static void Terminate();

	static void CreateLineEntity(const string& tag);
	static void CreateTriangleEntity(const string& tag);
	static void CreateBoxEntity(const string& tag);
	static void CreateWiredBoxEntity(const string& tag);
	static void CreateSphereEntity(const string& tag);
	static void CreateTextBlockEntity(const string& tag);

	static void Clear(const string& tag);
	static void ClearAll();
	static void SetVisiblility(bool visible, const string& tag);
	static void SetVisiblilityAll(bool visible);
	static void ToggleVisibility(const string& tag);
	static void ToggleVisibilityAll();

	static void AddLine(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c);
	static void AddLine(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c0, const glm::vec4& c1);

	static void AddTriangle(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec4& c);
	static void AddTriangle(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec4& c0, const glm::vec4& c1, const glm::vec4& c2);

	static void AddBox(const string& tag, const AABB& aabb, const glm::vec4& color);
	static void AddBox(const string& tag, const glm::vec3& center, const glm::vec3& normal, const glm::vec3& dimensions, const glm::vec4& color);

	static void AddWiredBox(const string& tag, const AABB& aabb, const glm::vec4& color);
	static void AddWiredBox(const string& tag, const glm::vec3& center, const glm::vec3& normal, const glm::vec3& dimensions, const glm::vec4& color);

	static void AddSphere(const string& tag, const glm::vec3& center, const glm::vec3& normal, float radius, const glm::vec4& color);

	static void AddText(const string& tag, const string& text = "", const glm::vec3& position = glm::vec3(0.0f), const glm::vec4& color = Color::black(), float fontSize = 32.0f);

	static bool AddToSelectionList(const string& tag);
	static unsigned int ShowNextSelection();
	static unsigned int ShowPreviousSelection();

private:
	static bool initialized;
	static map<string, Entity> entities;
	static map<string, DebuggingRenderable*> debuggingRenderables;
	static map<string, TextBlock*> textBlocks;

	static vector<string> selectionRenderables;
	static size_t selectionIndex;
};
