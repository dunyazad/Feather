#pragma once

#include <System/SystemBase.h>
#include <Component/GUIComponent/GUIComponents.h>

class GUISystem
{
public:
	GUISystem(FeatherWindow* window);
	~GUISystem();

	virtual void Initialize();
	virtual void Terminate();
	virtual void Update(ui32 frameNo, f32 timeDelta);

	void ReloadFont(float newFontSize);

	void ShowGraphPanel();
	void ShowUIPanel();
	void ShowStatusPanel(StatusPanel& panel);

	inline float GetFontSize() const { return fontSize; }
	inline void SetFontSize(float size) { fontSize = size; }

private:
	FeatherWindow* window;
	float fontSize = 20.0f;
	bool needFontReload = true;
};
