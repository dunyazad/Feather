#include <Component/Renderable.h>
#include <Component/Shader.h>

Renderable::Renderable()
{
}

Renderable::~Renderable()
{
	if (vao != UINT32_MAX)
	{
		glDeleteVertexArrays(1, &vao);
		vao = UINT32_MAX;
	}
	// Release all buffers (assumes GraphicsBuffer<T> has a .Destroy() or similar)
	indices.Terminate();
	vertices.Terminate();
	normals.Terminate();
	colors3.Terminate();
	colors4.Terminate();
	uvs.Terminate();

	instanceColors.Terminate();
	instanceNormals.Terminate();
	instanceTransforms.Terminate();
}

void Renderable::Initialize(GeometryMode geometryMode)
{
	this->geometryMode = geometryMode;

	glGenVertexArrays(1, &vao);

	glBindVertexArray(vao);

	indices.Initialize(UINT32_MAX, GraphicsBuffer<ui32>::BufferTarget::Element);
	vertices.Initialize(0, GraphicsBuffer<glm::vec3>::BufferTarget::Array);
	normals.Initialize(1, GraphicsBuffer<glm::vec3>::BufferTarget::Array);
	colors3.Initialize(2, GraphicsBuffer<glm::vec3>::BufferTarget::Array);
	colors4.Initialize(2, GraphicsBuffer<glm::vec4>::BufferTarget::Array);
	uvs.Initialize(3, GraphicsBuffer<glm::vec2>::BufferTarget::Array);

	glBindVertexArray(0);
}

void Renderable::EnableInstancing(ui32 numberOfInstances)
{
	this->numberOfInstances = numberOfInstances;

	glBindVertexArray(vao);

	instanceColors.Initialize(4, GraphicsBuffer<glm::vec4>::BufferTarget::Array);
	instanceColors.SetUseInstancing(true);

	instanceNormals.Initialize(5, GraphicsBuffer<glm::vec3>::BufferTarget::Array);
	instanceNormals.SetUseInstancing(true);

	instanceTransforms.Initialize(6, GraphicsBuffer<glm::mat4>::BufferTarget::Array);
	instanceTransforms.SetUseInstancing(true);

	glBindVertexArray(0);
}

void Renderable::Update(ui32 frameNo, f32 timeDelta)
{
	if (false == dirty) return;

	if (UINT_MAX == vao)
	{
		Initialize(geometryMode);
	}

	glBindVertexArray(vao);

	indices.Update();
	vertices.Update();
	normals.Update();
	colors3.Update();
	colors4.Update();
	uvs.Update();

	if (numberOfInstances > 0)
	{
		instanceColors.Update();
		instanceNormals.Update();
		instanceTransforms.Update();
	}

	glBindVertexArray(0);

	dirty = false;
}

void Renderable::Draw(Shader* shader)
{
	if (false == visible) return;

	glBindVertexArray(vao);

	if (Solid == drawingMode || WireFrame == drawingMode || WireFrameSingleColor == drawingMode)
	{
		if (WireFrame == drawingMode || WireFrameSingleColor == drawingMode)
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		}

		if (WireFrameSingleColor == drawingMode)
		{
			{
				auto index = shader->GetUniformLocation("useSolidColor");
				if (-1 != index)
				{
					shader->UniformInt(index, 1);
				}
			}
			{
				auto index = shader->GetUniformLocation("solidColor");
				if (-1 != index)
				{
					shader->UniformV3(index, { 0.25f, 0.25f, 0.25f });
				}
			}
		}

		if (0 < numberOfInstances)
		{
			if (0 < indices.size())
			{
				glDrawElementsInstanced(geometryMode, indices.size(), GL_UNSIGNED_INT, nullptr, numberOfInstances);
			}
			else
			{
				glDrawArraysInstanced(geometryMode, 0, vertices.size(), numberOfInstances);
			}
		}
		else
		{
			if (0 != indices.size())
			{
				glDrawElements(geometryMode, indices.size(), GL_UNSIGNED_INT, nullptr);
			}
			else
			{
				glDrawArrays(geometryMode, 0, vertices.size());
			}
		}

		if (WireFrame == drawingMode || WireFrameSingleColor == drawingMode)
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}

		if (WireFrameSingleColor == drawingMode)
		{
			auto index = shader->GetUniformLocation("useSolidColor");
			if (-1 != index)
			{
				shader->UniformInt(index, 0);
			}
		}
	}
	else if (WireFrameOverSolid == drawingMode)
	{
		glLineWidth(2.0f);

		// Enable depth testing and render solid mesh
		glEnable(GL_DEPTH_TEST);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		// Apply polygon offset to push filled mesh back slightly
		glEnable(GL_POLYGON_OFFSET_FILL);
		glPolygonOffset(1.0f, 1.0f);  // Push solid mesh back

		if (0 < numberOfInstances)
		{
			if (0 < indices.size())
			{
				glDrawElementsInstanced(geometryMode, indices.size(), GL_UNSIGNED_INT, nullptr, numberOfInstances);
			}
			else
			{
				glDrawArraysInstanced(geometryMode, 0, vertices.size(), numberOfInstances);
			}
		}
		else
		{
			if (0 != indices.size())
			{
				glDrawElements(geometryMode, indices.size(), GL_UNSIGNED_INT, nullptr);
			}
			else
			{
				glDrawArrays(geometryMode, 0, vertices.size());
			}
		}

		// Disable polygon offset for wireframe rendering
		glDisable(GL_POLYGON_OFFSET_FILL);

		// Render wireframe on top with depth test enabled
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glEnable(GL_POLYGON_OFFSET_LINE);
		glPolygonOffset(0.5f, 0.5f);  // Slight offset to avoid z-fighting
		glEnable(GL_DEPTH_TEST);  // Keep depth testing ON

		{
			auto index = shader->GetUniformLocation("useSolidColor");
			if (-1 != index)
			{
				shader->UniformInt(index, 1);
			}
		}
		{
			auto index = shader->GetUniformLocation("solidColor");
			if (-1 != index)
			{
				shader->UniformV3(index, { 0.25f, 0.25f, 0.25f });
			}
		}

		if (0 < numberOfInstances)
		{
			if (0 < indices.size())
			{
				glDrawElementsInstanced(geometryMode, indices.size(), GL_UNSIGNED_INT, nullptr, numberOfInstances);
			}
			else
			{
				glDrawArraysInstanced(geometryMode, 0, vertices.size(), numberOfInstances);
			}
		}
		else
		{
			if (0 != indices.size())
			{
				glDrawElements(geometryMode, indices.size(), GL_UNSIGNED_INT, nullptr);
			}
			else
			{
				glDrawArrays(geometryMode, 0, vertices.size());
			}
		}

		{
			auto index = shader->GetUniformLocation("useSolidColor");
			if (-1 != index)
			{
				shader->UniformInt(index, 0);
			}
		}

		// Restore default polygon mode
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glLineWidth(2.0f);
	}
}

void Renderable::Clear()
{
	indices.Clear();
	vertices.Clear();
	normals.Clear();
	colors3.Clear();
	colors4.Clear();
	uvs.Clear();

	instanceTransforms.Clear();
	instanceColors.Clear();
	instanceNormals.Clear();

	numberOfInstances = 1;
}

void Renderable::AddIndex(ui32 index)
{
	dirty = true;

	indices.AddData(index);
}

void Renderable::AddVertex(const glm::vec3& vertex)
{
	dirty = true;

	vertices.AddData(vertex);
}

void Renderable::AddNormal(const glm::vec3& normal)
{
	dirty = true;

	normals.AddData(normal);
}

void Renderable::AddColor(const glm::vec3& color)
{
	dirty = true;

	colors3.AddData(color);
}

void Renderable::AddColor(const glm::vec4& color)
{
	dirty = true;

	colors4.AddData(color);
}

void Renderable::AddUV(const glm::vec2& uv)
{
	dirty = true;

	uvs.AddData(uv);
}

ui32 Renderable::GetIndex(ui32 bufferIndex)
{
	if (bufferIndex >= indices.size()) return UINT32_MAX;
	return indices[bufferIndex];
}

glm::vec3& Renderable::GetVertex(ui32 bufferIndex)
{
	if (bufferIndex >= vertices.size()) return glm::vec3();
	return vertices[bufferIndex];
}

glm::vec3& Renderable::GetNormal(ui32 bufferIndex)
{
	if (bufferIndex >= normals.size()) return glm::vec3();
	return normals[bufferIndex];
}

glm::vec3& Renderable::GetColor3(ui32 bufferIndex)
{
	if (bufferIndex >= colors3.size()) return glm::vec3();
	return colors3[bufferIndex];
}

glm::vec4& Renderable::GetColor4(ui32 bufferIndex)
{
	if (bufferIndex >= colors4.size()) return glm::vec4();
	return colors4[bufferIndex];
}

glm::vec2& Renderable::GetUV(ui32 bufferIndex)
{
	if (bufferIndex >= uvs.size()) return glm::vec2();
	return uvs[bufferIndex];
}

void Renderable::SetIndex(ui32 bufferIndex, ui32 index)
{
	dirty = true;

	indices.SetData(bufferIndex, index);
}

void Renderable::SetVertex(ui32 bufferIndex, const glm::vec3& vertex)
{
	dirty = true;

	vertices.SetData(bufferIndex, vertex);
}

void Renderable::SetNormal(ui32 bufferIndex, const glm::vec3& normal)
{
	dirty = true;

	normals.SetData(bufferIndex, normal);
}

void Renderable::SetColor(ui32 bufferIndex, const glm::vec3& color)
{
	dirty = true;

	colors3.SetData(bufferIndex, color);
}

void Renderable::SetColor(ui32 bufferIndex, const glm::vec4& color)
{
	dirty = true;

	colors4.SetData(bufferIndex, color);
}

void Renderable::SetUV(ui32 bufferIndex, const glm::vec2& uv)
{
	dirty = true;

	uvs.SetData(bufferIndex, uv);
}

void Renderable::AddInstanceColor(const glm::vec4& color)
{
	dirty = true;

	instanceColors.AddData(color);
}

void Renderable::AddInstanceNormal(const glm::vec3& normal)
{
	dirty = true;

	instanceNormals.AddData(normal);
}

void Renderable::AddInstanceTransform(const glm::mat4& transform)
{
	dirty = true;

	instanceTransforms.AddData(transform);
}

const glm::vec4& Renderable::GetInstanceColor(ui32 bufferIndex) const
{
	return instanceColors.at(bufferIndex);
}

void Renderable::SetInstanceColor(ui32 bufferIndex, const glm::vec4& color)
{
	dirty = true;

	instanceColors.SetData(bufferIndex, color);
}

const glm::vec3& Renderable::GetInstanceNormal(ui32 bufferIndex) const
{
	return instanceNormals.at(bufferIndex);
}

void Renderable::SetInstanceNormal(ui32 bufferIndex, const glm::vec3& normal)
{
	dirty = true;

	instanceNormals.SetData(bufferIndex, normal);
}

const glm::mat4& Renderable::GetInstanceTransform(ui32 bufferIndex) const
{
	return instanceTransforms.at(bufferIndex);
}

void Renderable::SetInstanceTransform(ui32 bufferIndex, const glm::mat4& transform)
{
	dirty = true;

	instanceTransforms.SetData(bufferIndex, transform);
}

void Renderable::AddIndices(const vector<ui32>& indices)
{
	dirty = true;

	this->indices.AddData(indices.data(), indices.size());
}

void Renderable::AddIndices(const ui32* indices, ui32 numberOfElements)
{
	dirty = true;

	this->indices.AddData(indices, numberOfElements);
}

void Renderable::AddVertices(const vector<glm::vec3>& vertices)
{
	dirty = true;

	this->vertices.AddData(vertices.data(), vertices.size());
}

void Renderable::AddVertices(const glm::vec3* vertices, ui32 numberOfElements)
{
	dirty = true;

	this->vertices.AddData(vertices, numberOfElements);
}

void Renderable::AddNormals(const vector<glm::vec3>& normals)
{
	dirty = true;

	this->normals.AddData(normals.data(), normals.size());
}

void Renderable::AddNormals(const glm::vec3* normals, ui32 numberOfElements)
{
	dirty = true;

	this->normals.AddData(normals, numberOfElements);
}

void Renderable::AddColors(const vector<glm::vec3>& colors)
{
	dirty = true;

	this->colors3.AddData(colors.data(), colors.size());
}

void Renderable::AddColors(const glm::vec3* colors, ui32 numberOfElements)
{
	dirty = true;

	this->colors3.AddData(colors, numberOfElements);
}

void Renderable::AddColors(const vector<glm::vec4>& colors)
{
	dirty = true;

	this->colors4.AddData(colors.data(), colors.size());
}

void Renderable::AddColors(const glm::vec4* colors, ui32 numberOfElements)
{
	dirty = true;

	this->colors4.AddData(colors, numberOfElements);
}

void Renderable::AddUVs(const vector<glm::vec2>& uvs)
{
	dirty = true;

	this->uvs.AddData(uvs.data(), uvs.size());
}

void Renderable::AddUVs(const glm::vec2* uvs, ui32 numberOfElements)
{
	dirty = true;

	this->uvs.AddData(uvs, numberOfElements);
}

void Renderable::AddInstanceColors(const vector<glm::vec4>& colors)
{
	dirty = true;

	this->instanceColors.AddData(colors.data(), colors.size());
}

void Renderable::AddInstanceColors(const glm::vec4* colors, ui32 numberOfElements)
{
	dirty = true;

	this->instanceColors.AddData(colors, numberOfElements);
}

void Renderable::AddInstanceNormals(const vector<glm::vec3>& normals)
{
	dirty = true;

	this->instanceNormals.AddData(normals.data(), normals.size());
}

void Renderable::AddInstanceNormals(const glm::vec3* normals, ui32 numberOfElements)
{
	dirty = true;

	this->instanceNormals.AddData(normals, numberOfElements);
}

void Renderable::AddInstanceTransforms(const vector<glm::mat4>& transforms)
{
	dirty = true;

	this->instanceTransforms.AddData(transforms.data(), transforms.size());
}

void Renderable::AddInstanceTransforms(const glm::mat4* transforms, ui32 numberOfElements)
{
	dirty = true;

	this->instanceTransforms.AddData(transforms, numberOfElements);
}
