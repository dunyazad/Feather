#pragma once

#include<FeatherCommon.h>

class FrameBuffer
{
protected:
    GLuint fbo = 0;
    GLuint colorTex = 0;
    GLuint depthTex = 0;
    int width = 0;
    int height = 0;

public:
    FrameBuffer() = default;

    void Initialize(int w, int h, bool withColor = true);
    void Terminate();

    void Bind();
    void Unbind();

    void Resize(int w, int h);

    void SaveDepth(const std::string& filename);

    inline GLuint GetFbo() { return fbo;}
    inline GLuint GetColorTex() { return colorTex; }
    inline GLuint GetDepthTex() { return depthTex; }
    inline int GetWidth() { return width;}
    inline int GetHeight() { return height; }

private:
    static void DepthToColor(float d, unsigned char& r, unsigned char& g, unsigned char& b);
};