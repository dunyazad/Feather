#pragma once

#include <FeatherCommon.h>

class FeatherWindow;
class Shader;
class CameraBase;
class Renderable;
class DebuggingRenderable;

class RenderSystem
{
public:
	RenderSystem(FeatherWindow* window);
	~RenderSystem();

	virtual void Initialize();
	virtual void Terminate();
	virtual void Update(ui32 frameNo, f32 timeDelta);

	void RenderRenderables(
		ui32 frameNo, f32 timeDelta,
		const glm::mat4& viewMatrix,
		const glm::mat4& perspectiveMatrix,
		const glm::vec3& eye,
		const map<Shader*, vector<Renderable*>>& shaderMapping);

	void RenderDebuggingRenderables(
		ui32 frameNo, f32 timeDelta,
		const glm::mat4& viewMatrix,
		const glm::mat4& perspectiveMatrix,
		const glm::vec3& eye,
		const map<Shader*, vector<DebuggingRenderable*>>& shaderMapping);

private:
	FeatherWindow* window = nullptr;
	CameraBase* activeCamera = nullptr;

	f32 fontSize = 18.0f;
	bool needFontReload = false;
};
