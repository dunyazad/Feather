#pragma once

#include <FeatherCommon.h>

class Texture
{
public:
	Texture();
	~Texture();

	void Bind();
	void Unbind();

	void SetTextureData(ui32 width = 1024, ui32 height = 1024, ui8* data = nullptr);

private:
	GLuint textureID = ui32_max;
	ui32 width = 1024;
	ui32 height = 1024;
	ui8* data = nullptr;
};
