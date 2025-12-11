#pragma once

#include <FeatherCommon.h>

class TreeViewPanel
{
public:
    TreeViewPanel();
	~TreeViewPanel();

	virtual void Render();
    
	inline bool IsVisible() const { return visible; }
	inline void SetVisible(bool visible) { this->visible = visible; }
	inline void ToggleVisible() { visible = !visible; }

protected:
	bool visible = true;
};
