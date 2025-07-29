#pragma once

#include <FeatherCommon.h>

class DebuggingRenderable;

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

	static void Clear(const string& tag);
	static void AddLine(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c0, const glm::vec4& c1);
	static void AddTriangle(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec4& c0, const glm::vec4& c1, const glm::vec4& c2);
	static void AddBox(const string& tag, const glm::vec3& center, const glm::vec3& normal, const glm::vec3& dimensions, const glm::vec4& color);

private:
	static bool initialized;
	static map<string, Entity> entities;
	static map<string, DebuggingRenderable*> debuggingRenderables;
};
