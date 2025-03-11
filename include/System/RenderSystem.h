#pragma once

#include <System/SystemBase.h>
#include <Core/Shader.h>

class RenderSystem : public SystemBase
{
public:
	RenderSystem(FeatherWindow* window);
	~RenderSystem();

	virtual void Initialize() override;
	virtual void Terminate() override;
	virtual void Update(ui32 frameNo, f32 timeDelta) override;

	Shader* CreateShader();

private:
	f32 fontSize = 18.0f;
	bool needFontReload = false;

	vector<Shader*> shaders;
};
