#include <Component/Renderable.h>

Renderable::Renderable()
{
}

Renderable::~Renderable()
{
}

void Renderable::Initialize(GeometryMode geometryMode)
{
	this->geometryMode = geometryMode;

	glGenVertexArrays(1, &vao);

	glBindVertexArray(vao);

	indices.Initialize(UINT32_MAX, GraphicsBuffer<ui32>::BufferTarget::Element);
	vertices.Initialize(0, GraphicsBuffer<MiniMath::V3>::BufferTarget::Array);
	normals.Initialize(1, GraphicsBuffer<MiniMath::V3>::BufferTarget::Array);
	colors3.Initialize(2, GraphicsBuffer<MiniMath::V3>::BufferTarget::Array);
	colors4.Initialize(2, GraphicsBuffer<MiniMath::V4>::BufferTarget::Array);
	uvs.Initialize(3, GraphicsBuffer<MiniMath::V2>::BufferTarget::Array);

	glBindVertexArray(0);
}

void Renderable::EnableInstancing(ui32 numberOfInstances)
{
	this->numberOfInstances = numberOfInstances;

	glBindVertexArray(vao);

	instanceColors.Initialize(4, GraphicsBuffer<MiniMath::V4>::BufferTarget::Array);
	instanceColors.SetUseInstancing(true);

	instanceNormals.Initialize(5, GraphicsBuffer<MiniMath::V3>::BufferTarget::Array);
	instanceNormals.SetUseInstancing(true);

	instanceTransforms.Initialize(6, GraphicsBuffer<MiniMath::M4>::BufferTarget::Array);
	instanceTransforms.SetUseInstancing(true);

	glBindVertexArray(0);
}

void Renderable::Update(ui32 frameNo, f32 timeDelta)
{
	if (false == needToUpdate) return;

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

	if (numberOfInstances > 1)
	{
		instanceColors.Update();
		instanceNormals.Update();
		instanceTransforms.Update();
	}

	glBindVertexArray(0);

	needToUpdate = false;
}

void Renderable::Draw()
{
	if (false == visible) return;

	glBindVertexArray(vao);

	if (Solid == drawingMode || WireFrame == drawingMode)
	{
		if (WireFrame == drawingMode)
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
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

		if (WireFrame == drawingMode)
		{
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		}
	}
	else if (WireFrameOverSolid == drawingMode)
	{
		glLineWidth(20.0f);

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

		// Restore default polygon mode
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

		glLineWidth(2.0f);
	}
}

void Renderable::AddIndex(ui32 index)
{
	needToUpdate = true;

	indices.AddData(index);
}

void Renderable::AddVertex(const MiniMath::V3& vertex)
{
	needToUpdate = true;

	vertices.AddData(vertex);
}

void Renderable::AddNormal(const MiniMath::V3& normal)
{
	needToUpdate = true;

	normals.AddData(normal);
}

void Renderable::AddColor(const MiniMath::V3& color)
{
	needToUpdate = true;

	colors3.AddData(color);
}

void Renderable::AddColor(const MiniMath::V4& color)
{
	needToUpdate = true;

	colors4.AddData(color);
}

void Renderable::AddUV(const MiniMath::V2& uv)
{
	needToUpdate = true;

	uvs.AddData(uv);
}

void Renderable::SetIndex(ui32 bufferIndex, ui32 index)
{
	needToUpdate = true;

	indices.SetData(bufferIndex, index);
}

void Renderable::SetVertex(ui32 bufferIndex, const MiniMath::V3& vertex)
{
	needToUpdate = true;

	vertices.SetData(bufferIndex, vertex);
}

void Renderable::SetNormal(ui32 bufferIndex, const MiniMath::V3& normal)
{
	needToUpdate = true;

	normals.SetData(bufferIndex, normal);
}

void Renderable::SetColor(ui32 bufferIndex, const MiniMath::V3& color)
{
	needToUpdate = true;

	colors3.SetData(bufferIndex, color);
}

void Renderable::SetColor(ui32 bufferIndex, const MiniMath::V4& color)
{
	needToUpdate = true;

	colors4.SetData(bufferIndex, color);
}

void Renderable::SetUV(ui32 bufferIndex, const MiniMath::V2& uv)
{
	needToUpdate = true;

	uvs.SetData(bufferIndex, uv);
}

void Renderable::AddInstanceColor(const MiniMath::V4& color)
{
	needToUpdate = true;

	instanceColors.AddData(color);
}

void Renderable::AddInstanceNormal(const MiniMath::V3& normal)
{
	needToUpdate = true;

	instanceNormals.AddData(normal);
}

void Renderable::AddInstanceTransform(const MiniMath::M4& transform)
{
	needToUpdate = true;

	instanceTransforms.AddData(transform);
}

void Renderable::SetInstanceColor(ui32 bufferIndex, const MiniMath::V4& color)
{
	needToUpdate = true;

	instanceColors.SetData(bufferIndex, color);
}

void Renderable::SetInstanceNormal(ui32 bufferIndex, const MiniMath::V3& normal)
{
	needToUpdate = true;

	instanceNormals.SetData(bufferIndex, normal);
}

void Renderable::SetInstanceTransform(ui32 bufferIndex, const MiniMath::M4& transform)
{
	needToUpdate = true;

	instanceTransforms.SetData(bufferIndex, transform);
}

void Renderable::AddIndices(const vector<ui32>& indices)
{
	needToUpdate = true;

	this->indices.AddData(indices.data(), indices.size());
}

void Renderable::AddIndices(const ui32* indices, ui32 numberOfElements)
{
	needToUpdate = true;

	this->indices.AddData(indices, numberOfElements);
}

void Renderable::AddVertices(const vector<MiniMath::V3>& vertices)
{
	needToUpdate = true;

	this->vertices.AddData(vertices.data(), vertices.size());
}

void Renderable::AddVertices(const MiniMath::V3* vertices, ui32 numberOfElements)
{
	needToUpdate = true;

	this->vertices.AddData(vertices, numberOfElements);
}

void Renderable::AddNormals(const vector<MiniMath::V3>& normals)
{
	needToUpdate = true;

	this->normals.AddData(normals.data(), normals.size());
}

void Renderable::AddNormals(const MiniMath::V3* normals, ui32 numberOfElements)
{
	needToUpdate = true;

	this->normals.AddData(normals, numberOfElements);
}

void Renderable::AddColors(const vector<MiniMath::V3>& colors)
{
	needToUpdate = true;

	this->colors3.AddData(colors.data(), colors.size());
}

void Renderable::AddColors(const MiniMath::V3* colors, ui32 numberOfElements)
{
	needToUpdate = true;

	this->colors3.AddData(colors, numberOfElements);
}

void Renderable::AddColors(const vector<MiniMath::V4>& colors)
{
	needToUpdate = true;

	this->colors4.AddData(colors.data(), colors.size());
}

void Renderable::AddColors(const MiniMath::V4* colors, ui32 numberOfElements)
{
	needToUpdate = true;

	this->colors4.AddData(colors, numberOfElements);
}

void Renderable::AddUVs(const vector<MiniMath::V2>& uvs)
{
	needToUpdate = true;

	this->uvs.AddData(uvs.data(), uvs.size());
}

void Renderable::AddUVs(const MiniMath::V2* uvs, ui32 numberOfElements)
{
	needToUpdate = true;

	this->uvs.AddData(uvs, numberOfElements);
}

void Renderable::AddInstanceColors(const vector<MiniMath::V4>& colors)
{
	needToUpdate = true;

	this->instanceColors.AddData(colors.data(), colors.size());
}

void Renderable::AddInstanceColors(const MiniMath::V4* colors, ui32 numberOfElements)
{
	needToUpdate = true;

	this->instanceColors.AddData(colors, numberOfElements);
}

void Renderable::AddInstanceNormals(const vector<MiniMath::V3>& normals)
{
	needToUpdate = true;

	this->instanceNormals.AddData(normals.data(), normals.size());
}

void Renderable::AddInstanceNormals(const MiniMath::V3* normals, ui32 numberOfElements)
{
	needToUpdate = true;

	this->instanceNormals.AddData(normals, numberOfElements);
}

void Renderable::AddInstanceTransforms(const vector<MiniMath::M4>& transforms)
{
	needToUpdate = true;

	this->instanceTransforms.AddData(transforms.data(), transforms.size());
}

void Renderable::AddInstanceTransforms(const MiniMath::M4* transforms, ui32 numberOfElements)
{
	needToUpdate = true;

	this->instanceTransforms.AddData(transforms, numberOfElements);
}
