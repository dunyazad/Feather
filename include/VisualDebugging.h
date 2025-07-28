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

	static void CreateEntity(const string& tag);
	static void Clear(const string& tag);
	static void AddLine(const string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c0, const glm::vec4& c1);

private:
	static bool initialized;
	static map<string, Entity> entities;
	static map<string, DebuggingRenderable*> debuggingRenderables;
};
