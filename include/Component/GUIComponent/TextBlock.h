#pragma once

#include <FeatherCommon.h>

struct TextInfo
{
	string text = "";
	glm::vec3 position = glm::vec3(0.0f);
	glm::vec4 color = Color::black();
	float fontSize = 32.0f;
};

class TextBlock
{
public:
    TextBlock();
	~TextBlock();

	virtual void Render();
    
	void AddText(const string& text = "", const glm::vec3& position = glm::vec3(0.0f), const glm::vec4& color = Color::black(), float fontSize = 32.0f);

	void Clear();

	inline bool IsVisible() const { return visible; }
	inline void SetVisible(bool visible) { this->visible = visible; }
	inline void ToggleVisible() { visible = !visible; }

protected:
	bool visible = true;
	vector<TextInfo> textInfos;
};
