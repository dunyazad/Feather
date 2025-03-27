#pragma once

#include <FeatherCommon.h>

#include <Component/ComponentBase.h>

class Shader;

template<typename T>
class GraphicsBuffer
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

	void SetData(ui32 bufferIndex, const T& data)
	{
		if (bufferIndex >= datas.size() - 1) return;

		datas[bufferIndex] = data;

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

			if constexpr (is_same_v<T, ui32>) {
				glVertexAttribIPointer(attributeIndex, 1, GL_UNSIGNED_INT, sizeof(T), (void*)0);
				if (useInstancing)
				{
					glVertexAttribDivisor(attributeIndex, 1); // Set attribute to be per-instance
				}
			}
			else if constexpr (is_same_v<T, MiniMath::V3>) {
				glVertexAttribPointer(attributeIndex, 3, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
				if (useInstancing)
				{
					glVertexAttribDivisor(attributeIndex, 1); // Set attribute to be per-instance
				}
			}
			else if constexpr (is_same_v<T, MiniMath::V4>) {
				glVertexAttribPointer(attributeIndex, 4, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
				if (useInstancing)
				{
					glVertexAttribDivisor(attributeIndex, 1); // Set attribute to be per-instance
				}
			}
			else if constexpr (is_same_v<T, MiniMath::M4>) {
				if (useInstancing)
				{
					for (int i = 0; i < 4; i++) {
						glVertexAttribPointer(attributeIndex + i, 4, GL_FLOAT, GL_FALSE, sizeof(T), (void*)(sizeof(float) * i * 4));
						glEnableVertexAttribArray(attributeIndex + i);
						glVertexAttribDivisor(attributeIndex + i, 1); // Set attribute to be per-instance
					}
				}
				else
				{
					glVertexAttribPointer(attributeIndex, 16, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
				}
			}
			else {
				glVertexAttribPointer(attributeIndex, sizeof(T) / sizeof(float), GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
			}

			glEnableVertexAttribArray(attributeIndex);
			needToUpdate = false;
		}
	}

	inline ui64 size() { return datas.size(); }
	inline bool empty() { return datas.empty(); }
	inline bool IsUseInstancing() { return useInstancing; }
	inline void SetUseInstancing(bool use) { useInstancing = use; }

protected:
	BufferTarget bufferTarget = Array;
	BufferUsage bufferUsage = Static;

	bool useInstancing = false;

	bool needToUpdate = true;
	GLuint attributeIndex = UINT32_MAX;
	vector<T> datas;
};

class Renderable : public RegisterDerivation<Renderable, ComponentBase>
{
public:
	enum GeometryMode
	{
		Points = GL_POINTS,
		Lines = GL_LINES,
		LineLoop = GL_LINE_LOOP,
		LineStrip = GL_LINE_STRIP,
		Triangles = GL_TRIANGLES,
		TriangleStrip = GL_TRIANGLE_STRIP,
		TriangleFan = GL_TRIANGLE_FAN,
		Quads = GL_QUADS
	};

	enum DrawingMode
	{
		Solid,
		WireFrameOverSolid,
		WireFrame,
		NumberOfDrawingModes
	};

public:
	Renderable();
	~Renderable();

	void Initialize(GeometryMode geometryMode);
	void EnableInstancing(ui32 numberOfInstances);

	virtual void Update(ui32 frameNo, f32 timeDelta);

	virtual void Draw();

	void AddIndex(ui32 index);
	void AddVertex(const MiniMath::V3& vertex);
	void AddNormal(const MiniMath::V3& normal);
	void AddColor(const MiniMath::V3& color);
	void AddColor(const MiniMath::V4& color);
	void AddUV(const MiniMath::V2& uv);

	void SetIndex(ui32 bufferIndex, ui32 index);
	void SetVertex(ui32 bufferIndex, const MiniMath::V3& vertex);
	void SetNormal(ui32 bufferIndex, const MiniMath::V3& normal);
	void SetColor(ui32 bufferIndex, const MiniMath::V3& color);
	void SetColor(ui32 bufferIndex, const MiniMath::V4& color);
	void SetUV(ui32 bufferIndex, const MiniMath::V2& uv);

	void AddInstanceColor(const MiniMath::V4& color);
	void AddInstanceNormal(const MiniMath::V3& normal);
	void AddInstanceTransform(const MiniMath::M4& transform);

	void SetInstanceColor(ui32 bufferIndex, const MiniMath::V4& color);
	void SetInstanceNormal(ui32 bufferIndex, const MiniMath::V3& normal);
	void SetInstanceTransform(ui32 bufferIndex, const MiniMath::M4& transform);

	void AddIndices(const vector<ui32>& indices);
	void AddIndices(const ui32* indices, ui32 numberOfElements);

	void AddVertices(const vector<MiniMath::V3>& vertices);
	void AddVertices(const MiniMath::V3* vertices, ui32 numberOfElements);

	void AddNormals(const vector<MiniMath::V3>& normals);
	void AddNormals(const MiniMath::V3* normals, ui32 numberOfElements);

	void AddColors(const vector<MiniMath::V3>& colors);
	void AddColors(const MiniMath::V3* colors, ui32 numberOfElements);

	void AddColors(const vector<MiniMath::V4>& colors);
	void AddColors(const MiniMath::V4* colors, ui32 numberOfElements);

	void AddUVs(const vector<MiniMath::V2>& uvs);
	void AddUVs(const MiniMath::V2* uvs, ui32 numberOfElements);

	void AddInstanceColors(const vector<MiniMath::V4>& colors);
	void AddInstanceColors(const MiniMath::V4* colors, ui32 numberOfElements);

	void AddInstanceNormals(const vector<MiniMath::V3>& normals);
	void AddInstanceNormals(const MiniMath::V3* normals, ui32 numberOfElements);

	void AddInstanceTransforms(const vector<MiniMath::M4>& transforms);
	void AddInstanceTransforms(const MiniMath::M4* transforms, ui32 numberOfElements);

	inline bool IsVisible() const { return visible; }
	inline void SetVisible(bool visible) { this->visible = visible; }
	inline void ToggleVisible() { visible = !visible; }

	inline Shader* GetActiveShader() const { if (shaders.empty()) return nullptr; else return shaders[activeShaderIndex]; }
	inline ui32 GetActiveShaderIndex() { return activeShaderIndex; }
	inline void SetActiveShaderIndex(ui32 index) { activeShaderIndex = index; }
	inline const vector<Shader*>& GetShaders() const { return shaders; }
	inline void AddShader(Shader* shader) { shaders.push_back(shader); }

	inline GeometryMode GetGeometryMode() { return geometryMode; }
	inline void SetGeometryMode(GeometryMode geometryMode) { this->geometryMode = geometryMode; }

	inline DrawingMode GetDrawingMode() { return drawingMode; }
	inline void SetDrawingMode(DrawingMode drawingMode) { this->drawingMode = drawingMode; }

	inline void NextDrawingMode() { drawingMode = (DrawingMode)((drawingMode + 1) % NumberOfDrawingModes); }

private:
	bool visible = true;
	bool needToUpdate = true;

	ui32 activeShaderIndex = 0;
	vector<Shader*> shaders;
	GLuint vao = UINT_MAX;

	GeometryMode geometryMode = Triangles;
	DrawingMode drawingMode = Solid;

	GraphicsBuffer<ui32> indices;
	GraphicsBuffer<MiniMath::V3> vertices;
	GraphicsBuffer<MiniMath::V3> normals;
	GraphicsBuffer<MiniMath::V3> colors3;
	GraphicsBuffer<MiniMath::V4> colors4;
	GraphicsBuffer<MiniMath::V2> uvs;

	GraphicsBuffer<MiniMath::M4> instanceTransforms;
	GraphicsBuffer<MiniMath::V4> instanceColors;
	GraphicsBuffer<MiniMath::V3> instanceNormals;

	ui32 numberOfInstances = 1;
};

class DebuggingRenderable : public RegisterDerivation<DebuggingRenderable, Renderable>
{

};