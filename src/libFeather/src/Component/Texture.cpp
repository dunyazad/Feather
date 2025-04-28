#include <Component/Texture.h>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

Texture::Texture()
{
	glGenTextures(1, &textureID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);  // Wrapping mode on S axis
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);  // Wrapping mode on T axis
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); // Minifying filter
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); // Magnifying filter
}

Texture::~Texture()
{
	glDeleteTextures(1, &textureID);

	if (nullptr != this->data)
	{
		delete this->data;
	}
}

void Texture::Bind()
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, textureID);
}

void Texture::Unbind()
{
	glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::LoadFile(const File& file)
{
	stbi_set_flip_vertically_on_load(true);

	int width, height, channels;
	unsigned char* imageData = stbi_load(file.GetFileName().c_str(), &width, &height, &channels, 4);
	if (imageData)
	{
		SetTextureData(width, height, imageData);
		stbi_image_free(imageData);
	}
	else
	{
		std::cerr << "Failed to load texture file!" << std::endl;
	}
}

void Texture::AllocTextureData(ui32 width, ui32 height)
{
	this->width = width;
	this->height = height;
	this->data = new ui8[width * height * 4];
	memset(this->data, 0, sizeof(ui8) * width * height * 4);

	Bind();

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

	glGenerateMipmap(GL_TEXTURE_2D);
}

void Texture::SetTextureData(ui32 width, ui32 height, ui8* data)
{
	assert(nullptr != data && "data is empty!\n");

	if (nullptr == this->data || this->width * this->height < width * height)
	{
		if (nullptr != this->data)
		{
			delete this->data;
		}

		this->width = width;
		this->height = height;
		this->data = new ui8[width * height * 4];
		memset(this->data, 0, sizeof(ui8) * width * height * 4);
	}
	memcpy(this->data, data, sizeof(ui8) * width * height * 4);

	Bind();

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
	
	glGenerateMipmap(GL_TEXTURE_2D);
}