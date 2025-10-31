#include <Buffer.h>

void FrameBuffer::Initialize(int w, int h, bool withColor)
{
	width = w;
	height = h;

	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	if (withColor)
	{
		glGenTextures(1, &colorTex);
		glBindTexture(GL_TEXTURE_2D, colorTex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTex, 0);
	}

	glGenTextures(1, &depthTex);
	glBindTexture(GL_TEXTURE_2D, depthTex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 0,
		GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTex, 0);

	if (withColor)
	{
		GLenum drawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, drawBuffers);
	}
	else
	{
		glDrawBuffer(GL_NONE);
		glReadBuffer(GL_NONE);
	}

	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
		printf("[FBO] Framebuffer incomplete! Status = 0x%x\n", status);
	else
		printf("[FBO] Created %dx%d (color=%s)\n", width, height, withColor ? "yes" : "no");

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBuffer::Terminate()
{
	if (colorTex) glDeleteTextures(1, &colorTex);
	if (depthTex) glDeleteTextures(1, &depthTex);
	if (fbo) glDeleteFramebuffers(1, &fbo);
	colorTex = depthTex = fbo = 0;
}

void FrameBuffer::Bind()
{
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glViewport(0, 0, width, height);
}

void FrameBuffer::Unbind()
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBuffer::Resize(int w, int h)
{
	if (width == w && height == h)
		return;

	width = w;
	height = h;

	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	// Color attachment 리사이즈
	if (colorTex)
	{
		glBindTexture(GL_TEXTURE_2D, colorTex);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0,
			GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, colorTex, 0);
	}

	// Depth attachment 리사이즈
	glBindTexture(GL_TEXTURE_2D, depthTex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32F, width, height, 0,
		GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthTex, 0);

	GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (status != GL_FRAMEBUFFER_COMPLETE)
		printf("[FBO] Resize incomplete! Status = 0x%x\n", status);
	else
		printf("[FBO] Resized to %dx%d\n", width, height);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBuffer::SaveDepth(const std::string& filename)
{
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	std::vector<float> depth(width * height);
	glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depth.data());

	// Flip Y
	std::vector<float> flipped(width * height);
	for (int y = 0; y < height; ++y)
		memcpy(&flipped[y * width],
			&depth[(height - 1 - y) * width],
			width * sizeof(float));

	// Auto normalize
	float minD = 1.0f, maxD = 0.0f;
	for (float v : flipped)
	{
		if (v < minD) minD = v;
		if (v > maxD) maxD = v;
	}
	float range = std::max(maxD - minD, 1e-6f);

	printf("[FBO] Depth range: %.6f ~ %.6f\n", minD, maxD);

	// Convert to pseudo-color (jet)
	std::vector<unsigned char> rgb(width * height * 3);
	for (int i = 0; i < width * height; ++i)
	{
		float d = (flipped[i] - minD) / range;
		unsigned char r, g, b;
		DepthToColor(d, r, g, b);
		rgb[i * 3 + 0] = r;
		rgb[i * 3 + 1] = g;
		rgb[i * 3 + 2] = b;
	}

	stbi_write_png(filename.c_str(), width, height, 3, rgb.data(), width * 3);
	printf("[FBO] Saved depth pseudo-color: %s\n", filename.c_str());

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void FrameBuffer::DepthToColor(float d, unsigned char& r, unsigned char& g, unsigned char& b)
{
	d = std::clamp(d, 0.0f, 1.0f);
	float r_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.75f)), 0.0f, 1.0f);
	float g_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.50f)), 0.0f, 1.0f);
	float b_f = std::clamp(1.5f - std::fabs(4.0f * (d - 0.25f)), 0.0f, 1.0f);
	r = static_cast<unsigned char>(r_f * 255.0f);
	g = static_cast<unsigned char>(g_f * 255.0f);
	b = static_cast<unsigned char>(b_f * 255.0f);
}
