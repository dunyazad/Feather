#include <Component/Texture.h>

Texture::Texture()
    : width(0), height(0), data(nullptr)
{
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // 텍스처 파라미터 설정 (반드시 바인드 이후)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
}

Texture::~Texture()
{
    glDeleteTextures(1, &textureID);

	SAFE_DELETE_ARRAY(this->data);
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
        std::cerr << "Failed to load texture file: " << file.GetFileName() << std::endl;
    }
}

void Texture::AllocTextureData(ui32 width, ui32 height)
{
    this->width = width;
    this->height = height;

    if (this->data)
        delete[] this->data;
    this->data = new ui8[width * height * 4];
    memset(this->data, 0, sizeof(ui8) * width * height * 4);

    Bind();

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    Unbind();
}

void Texture::SetTextureData(ui32 width, ui32 height, ui8* data)
{
    assert(data && "data is empty!\n");

    if (this->data == nullptr || this->width * this->height < width * height)
    {
        if (this->data)
            delete[] this->data;
        this->data = new ui8[width * height * 4];
    }
    this->width = width;
    this->height = height;
    memcpy(this->data, data, sizeof(ui8) * width * height * 4);

    Bind();
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    Unbind();
}
