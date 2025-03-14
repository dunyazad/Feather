#pragma once

#include <FeatherCommon.h>

#include <Component/ComponentBase.h>

class Shader;

template<typename T>
class Buffer
{
public:
	enum BufferTarget { Array = GL_ARRAY_BUFFER, Element = GL_ELEMENT_ARRAY_BUFFER };
	enum BufferUsage { Static = GL_STATIC_DRAW, Dynamic = GL_DYNAMIC_DRAW };

public:
	GLuint vbo = UINT32_MAX;

	void Initialize(GLuint attributeIndex, BufferTarget bufferTarget, BufferUsage bufferUsage = Static)
	{
		this->attributeIndex = attributeIndex;
		this->bufferTarget = bufferTarget;
		this->bufferUsage = bufferUsage;

		glGenBuffers(1, &vbo);
	}

	void Terminate()
	{
		if (UINT32_MAX != vbo)
		{
			glDeleteBuffers(1, &vbo);
		}
	}

	void Bind() { glBindBuffer(bufferTarget, vbo); }

	void AddData(const T& data)
	{
		datas.push_back(data);

		needToUpdate = true;
	}

	void AddData(const T* datas, ui32 numberOfDatas)
	{
		this->datas.insert(this->datas.end(), datas, datas + numberOfDatas);

		needToUpdate = true;
	}

	void Update()
	{
		if (datas.empty()) return;

		if (needToUpdate)
		{
			Bind();

			glBufferData(bufferTarget, sizeof(T) * datas.size(), datas.data(), bufferUsage);

			glVertexAttribPointer(attributeIndex, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(attributeIndex);

			needToUpdate = false;
		}
	}

	ui64 size() { return datas.size(); }

protected:
	BufferTarget bufferTarget = Array;
	BufferUsage bufferUsage = Static;

	bool needToUpdate = true;
	GLuint attributeIndex = UINT32_MAX;
	vector<T> datas;
};

class Renderable : public RegisterDerivation<Renderable, ComponentBase>
{
public:
	enum GeometryMode {
		Points = GL_POINTS,
		Lines = GL_LINES,
		LineLoop = GL_LINE_LOOP,
		LineStrip = GL_LINE_STRIP,
		Triangles = GL_TRIANGLES,
		TriangleStrip = GL_TRIANGLE_STRIP,
		TriangleFan = GL_TRIANGLE_FAN,
		Quads = GL_QUADS
	};

public:
	Renderable(ComponentID id);
	~Renderable();

	void Initialize(GeometryMode geometryMode);

	virtual void Update(ui32 frameNo, f32 timeDelta);

	virtual void Draw();

	void AddVertex(const MiniMath::V3& vertex);
	void AddNormal(const MiniMath::V3& normal);
	void AddColor(const MiniMath::V4& color);
	void AddIndex(ui32 index);

	inline Shader* GetShader() const { return shader; }
	inline void SetShader(Shader* shader) { this->shader = shader; }

private:
	Shader* shader = nullptr;
	GLuint vao = UINT_MAX;

	GeometryMode geometryMode = Triangles;

	Buffer<MiniMath::V3> vertices;
	Buffer<MiniMath::V3> normals;
	Buffer<MiniMath::V4> colors;
	Buffer<ui32> indices;
};
