#pragma once

#include <FeatherCommon.h>

struct ControlPanelButton
{
	string label;
	float width = 100.0f; // Default width
	float height = 30.0f; // Default height
	vector<function<void()>> callbacks;
};

class ControlPanel
{
public:
    ControlPanel(const string& title);
	~ControlPanel();

	virtual void Render();

	void AddButton(const string& label, float width, float height, const function<void()>& callback);

protected:
	string title = "Control Panel";
	vector<ControlPanelButton> buttons;
};
