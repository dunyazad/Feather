#pragma once

#include <FeatherCommon.h>

struct ControlPanelButton
{
	std::string label;
	float width = 100.0f; // Default width
	float height = 30.0f; // Default height
	std::vector<std::function<void()>> callbacks;
};

class ControlPanel
{
public:
    ControlPanel(const std::string& title);
	~ControlPanel();

	virtual void Render();

	void AddButton(const std::string& label, float width, float height, const std::function<void()>& callback);

protected:
	std::string title = "Control Panel";
	std::vector<ControlPanelButton> buttons;
};
