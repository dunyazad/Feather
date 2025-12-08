#pragma once

#include <FeatherCommon.h>
#include <Component/Components.h>

class FeatherWindow;

class RenderSystem
{
public:
	RenderSystem(FeatherWindow* window);
	~RenderSystem();

	void Initialize();
	void Terminate();

	void Update(ui32 frameNo, f32 timeDelta);

	void RenderRenderables(
		ui32 frameNo, f32 timeDelta,
		const glm::mat4& viewMatrix,
		const glm::mat4& perspectiveMatrix,
		const glm::vec3& eye,
		const glm::vec4& lightVector,
		const std::map<Shader*, std::vector<Renderable*>>& shadermap
	);

	// [수정] lightVector 인자(glm::vec4)가 추가되었습니다.
	void RenderDebuggingRenderables(
		ui32 frameNo, f32 timeDelta,
		const glm::mat4& viewMatrix,
		const glm::mat4& perspectiveMatrix,
		const glm::vec3& eye,
		const glm::vec4& lightVector,
		const std::map<Shader*, std::vector<DebuggingRenderable*>>& shadermap
	);

private:
	FeatherWindow* window = nullptr;
};