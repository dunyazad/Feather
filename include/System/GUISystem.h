#pragma once

#include <System/SystemBase.h>

class GUISystem : public RegisterDerivation<GUISystem, SystemBase>
{
public:
	GUISystem(FeatherWindow* window);
	~GUISystem();

	virtual void Initialize() override;
	virtual void Terminate() override;
	virtual void Update(ui32 frameNo, f32 timeDelta) override;

	void ReloadFont(float newFontSize);

	void ShowGraphPanel();
	void ShowUIPanel();
	void ShowFPS();

	inline float GetFontSize() const { return fontSize; }
	inline void SetFontSize(float size) { fontSize = size; }

private:
	float fontSize = 20.0f;
	bool needFontReload = true;
};
