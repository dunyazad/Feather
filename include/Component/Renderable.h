#pragma once

#include <FeatherCommon.h>

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

	void Clear()
	{
		datas.clear();

		dirty = true;
	}

	void Bind() { glBindBuffer(bufferTarget, vbo); }

	ui32 AddData(const T& data)
	{
		datas.push_back(data);

		dirty = true;

		return datas.size() - 1;
	}

	void SetData(ui32 bufferIndex, const T& data)
	{
		if (bufferIndex >= datas.size() - 1) return;

		datas[bufferIndex] = data;

		dirty = true;
	}

	void AddData(const T* datas, ui32 numberOfDatas)
	{
		this->datas.insert(this->datas.end(), datas, datas + numberOfDatas);

		dirty = true;
	}

	void Update()
	{
		if (datas.empty()) return;

		if (dirty)
		{
			Bind();

			glBufferData(bufferTarget, sizeof(T) * datas.size(), datas.data(), bufferUsage);

			if (ui32_max != attributeIndex)
			{
				if constexpr (is_same_v<T, ui32>) {
					glVertexAttribIPointer(attributeIndex, 1, GL_UNSIGNED_INT, sizeof(T), (void*)0);
					if (useInstancing)
					{
						glVertexAttribDivisor(attributeIndex, 1); // Set attribute to be per-instance
					}
				}
				else if constexpr (is_same_v<T, glm::vec2>) {
					glVertexAttribPointer(attributeIndex, 2, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
					if (useInstancing)
					{
						glVertexAttribDivisor(attributeIndex, 1); // Set attribute to be per-instance
					}
				}
				else if constexpr (is_same_v<T, glm::vec3>) {
					glVertexAttribPointer(attributeIndex, 3, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
					if (useInstancing)
					{
						glVertexAttribDivisor(attributeIndex, 1); // Set attribute to be per-instance
					}
				}
				else if constexpr (is_same_v<T, glm::vec4>) {
					glVertexAttribPointer(attributeIndex, 4, GL_FLOAT, GL_FALSE, sizeof(T), (void*)0);
					if (useInstancing)
					{
						glVertexAttribDivisor(attributeIndex, 1); // Set attribute to be per-instance
					}
				}
				else if constexpr (is_same_v<T, glm::mat4>) {
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
			}
			dirty = false;
		}
	}

	inline ui64 size() { return datas.size(); }
	inline bool empty() { return datas.empty(); }
	inline T& at(ui32 index) { return datas.at(index); }
	inline const T& at(ui32 index) const { return datas.at(index); }
	inline bool IsUseInstancing() { return useInstancing; }
	inline void SetUseInstancing(bool use) { useInstancing = use; }

	inline T& operator [](unsigned int index) { return datas.at(index); }

protected:
	BufferTarget bufferTarget = Array;
	BufferUsage bufferUsage = Static;

	bool useInstancing = false;

	bool dirty = true;
	GLuint attributeIndex = UINT32_MAX;
	vector<T> datas;
};

class Renderable
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
		WireFrameSingleColor,
		NumberOfDrawingModes
	};

public:
	Renderable();
	~Renderable();

	void Initialize(GeometryMode geometryMode);
	void EnableInstancing();

	virtual void Update(ui32 frameNo, f32 timeDelta);

	virtual void Draw(Shader* shader);

	virtual void Clear();
	virtual void ClearInstancingData();

	ui32 AddIndex(ui32 index);
	ui32 AddVertex(const glm::vec3& vertex);
	ui32 AddNormal(const glm::vec3& normal);
	ui32 AddColor(const glm::vec3& color);
	ui32 AddColor(const glm::vec4& color);
	ui32 AddUV(const glm::vec2& uv);

	ui32 GetIndex(ui32 bufferIndex);
	glm::vec3& GetVertex(ui32 bufferIndex);
	glm::vec3& GetNormal(ui32 bufferIndex);
	glm::vec3& GetColor3(ui32 bufferIndex);
	glm::vec4& GetColor4(ui32 bufferIndex);
	glm::vec2& GetUV(ui32 bufferIndex);

	void SetIndex(ui32 bufferIndex, ui32 index);
	void SetVertex(ui32 bufferIndex, const glm::vec3& vertex);
	void SetNormal(ui32 bufferIndex, const glm::vec3& normal);
	void SetColor(ui32 bufferIndex, const glm::vec3& color);
	void SetColor(ui32 bufferIndex, const glm::vec4& color);
	void SetUV(ui32 bufferIndex, const glm::vec2& uv);

	void AddInstanceColor(const glm::vec4& color);
	void AddInstanceNormal(const glm::vec3& normal);
	void AddInstanceTransform(const glm::mat4& transform);

	const glm::vec4& GetInstanceColor(ui32 bufferIndex) const;
	void SetInstanceColor(ui32 bufferIndex, const glm::vec4& color);
	const glm::vec3& GetInstanceNormal(ui32 bufferIndex) const;
	void SetInstanceNormal(ui32 bufferIndex, const glm::vec3& normal);
	const glm::mat4& GetInstanceTransform(ui32 bufferIndex) const;
	void SetInstanceTransform(ui32 bufferIndex, const glm::mat4& transform);

	void AddIndices(const vector<ui32>& indices);
	void AddIndices(const ui32* indices, ui32 numberOfElements);

	void AddVertices(const vector<glm::vec3>& vertices);
	void AddVertices(const glm::vec3* vertices, ui32 numberOfElements);

	void AddNormals(const vector<glm::vec3>& normals);
	void AddNormals(const glm::vec3* normals, ui32 numberOfElements);

	void AddColors(const vector<glm::vec3>& colors);
	void AddColors(const glm::vec3* colors, ui32 numberOfElements);

	void AddColors(const vector<glm::vec4>& colors);
	void AddColors(const glm::vec4* colors, ui32 numberOfElements);

	void AddUVs(const vector<glm::vec2>& uvs);
	void AddUVs(const glm::vec2* uvs, ui32 numberOfElements);

	void AddInstanceColors(const vector<glm::vec4>& colors);
	void AddInstanceColors(const glm::vec4* colors, ui32 numberOfElements);

	void AddInstanceNormals(const vector<glm::vec3>& normals);
	void AddInstanceNormals(const glm::vec3* normals, ui32 numberOfElements);

	void AddInstanceTransforms(const vector<glm::mat4>& transforms);
	void AddInstanceTransforms(const glm::mat4* transforms, ui32 numberOfElements);

	inline bool IsVisible() const { return visible; }
	inline void SetVisible(bool visible) { this->visible = visible; }
	inline void ToggleVisible() { visible = !visible; }
	inline bool IsUsingAlpha() const { return useAlpha; }
	inline void SetUseAlpha(bool useAlpha) { this->useAlpha = useAlpha; }

	inline Shader* GetActiveShader() const { if (shaders.empty() || activeShaderIndex >= shaders.size()) return nullptr; else return shaders[activeShaderIndex]; }
	inline ui32 GetActiveShaderIndex() { return activeShaderIndex; }
	inline void SetActiveShaderIndex(ui32 index) { activeShaderIndex = index; }
	inline const vector<Shader*>& GetShaders() const { return shaders; }
	inline void AddShader(Shader* shader) { shaders.push_back(shader); }

	inline GeometryMode GetGeometryMode() { return geometryMode; }
	inline void SetGeometryMode(GeometryMode geometryMode) { this->geometryMode = geometryMode; }

	inline DrawingMode GetDrawingMode() { return drawingMode; }
	inline void SetDrawingMode(DrawingMode drawingMode) { this->drawingMode = drawingMode; }

	inline void NextDrawingMode() { drawingMode = (DrawingMode)((drawingMode + 1) % NumberOfDrawingModes); }

	inline GraphicsBuffer<ui32>& GetIndices() { return indices; }
	inline GraphicsBuffer<glm::vec3>& GetVertices() { return vertices; }
	inline GraphicsBuffer<glm::vec3>& GetNormals() { return normals; }
	inline GraphicsBuffer<glm::vec3>& GetColors3() { return colors3; }
	inline GraphicsBuffer<glm::vec4>& GetColors4() { return colors4; }
	inline GraphicsBuffer<glm::vec2>& GetUvs() { return uvs; }
	inline GraphicsBuffer<glm::mat4>& GetInstanceTransforms() { return instanceTransforms; }
	inline GraphicsBuffer<glm::vec4>& GetInstanceColors() { return instanceColors; }
	inline GraphicsBuffer<glm::vec3>& GetInstanceNormals() { return instanceNormals; }

	inline const GraphicsBuffer<ui32>& GetIndices() const { return indices; }
	inline const GraphicsBuffer<glm::vec3>& GetVertices() const { return vertices; }
	inline const GraphicsBuffer<glm::vec3>& GetNormals() const { return normals; }
	inline const GraphicsBuffer<glm::vec3>& GetColors3() const { return colors3; }
	inline const GraphicsBuffer<glm::vec4>& GetColors4() const { return colors4; }
	inline const GraphicsBuffer<glm::vec2>& GetUvs() const { return uvs; }
	inline const GraphicsBuffer<glm::mat4>& GetInstanceTransforms() const { return instanceTransforms; }
	inline const GraphicsBuffer<glm::vec4>& GetInstanceColors() const { return instanceColors; }
	inline const GraphicsBuffer<glm::vec3>& GetInstanceNormals() const { return instanceNormals; }

	inline void IncreaseNumberOfInstances() { numberOfInstances++; if (false == instancingEnabled) EnableInstancing(); }

	inline bool IsInstancingEnabled() const { return instancingEnabled; }

	inline ui32 GetNumberOfInstances() const { return numberOfInstances; }
	inline void SetNumberOfInstances(ui32 n) { numberOfInstances = n; }

private:
	bool visible = true;
	bool dirty = true;
	bool useAlpha = false;

	ui32 activeShaderIndex = 0;
	vector<Shader*> shaders;
	GLuint vao = UINT_MAX;

	GeometryMode geometryMode = Triangles;
	DrawingMode drawingMode = Solid;

	GraphicsBuffer<ui32> indices;
	GraphicsBuffer<glm::vec3> vertices;
	GraphicsBuffer<glm::vec3> normals;
	GraphicsBuffer<glm::vec3> colors3;
	GraphicsBuffer<glm::vec4> colors4;
	GraphicsBuffer<glm::vec2> uvs;

	GraphicsBuffer<glm::mat4> instanceTransforms;
	GraphicsBuffer<glm::vec4> instanceColors;
	GraphicsBuffer<glm::vec3> instanceNormals;

	ui32 numberOfInstances = 0;

	bool instancingEnabled = false;
};

class DebuggingRenderable : public Renderable
{
public:
};
