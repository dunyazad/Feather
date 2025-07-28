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

	static void AddLine(const string& tag, const MiniMath::V3& v0, const MiniMath::V3& v1, const MiniMath::V4& c0, const MiniMath::V4& c1);

private:
	static bool initialized;
	static map<string, Entity> entities;
	static map<string, DebuggingRenderable*> debuggingRenderables;
};