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

	instanceTransforms.Initialize(4, GraphicsBuffer<MiniMath::M4>::BufferTarget::Array);

	glBindVertexArray(0);
}

void Renderable::Update(ui32 frameNo, f32 timeDelta)
{
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
		instanceTransforms.Update();
	}

	glBindVertexArray(0);
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
	indices.AddData(index);
}

void Renderable::AddVertex(const MiniMath::V3& vertex)
{
	vertices.AddData(vertex);
}

void Renderable::AddNormal(const MiniMath::V3& normal)
{
	normals.AddData(normal);
}

void Renderable::AddColor(const MiniMath::V3& color)
{
	colors3.AddData(color);
}

void Renderable::AddColor(const MiniMath::V4& color)
{
	colors4.AddData(color);
}

void Renderable::AddUV(const MiniMath::V2& uv)
{
	uvs.AddData(uv);
}

void Renderable::AddInstanceTransform(const MiniMath::M4& transform)
{
	instanceTransforms.AddData(transform);
}

void Renderable::AddIndices(const vector<ui32>& indices)
{
	this->indices.AddData(indices.data(), indices.size());
}

void Renderable::AddIndices(const ui32* indices, ui32 numberOfElements)
{
	this->indices.AddData(indices, numberOfElements);
}

void Renderable::AddVertices(const vector<MiniMath::V3>& vertices)
{
	this->vertices.AddData(vertices.data(), vertices.size());
}

void Renderable::AddVertices(const MiniMath::V3* vertices, ui32 numberOfElements)
{
	this->vertices.AddData(vertices, numberOfElements);
}

void Renderable::AddNormals(const vector<MiniMath::V3>& normals)
{
	this->normals.AddData(normals.data(), normals.size());
}

void Renderable::AddNormals(const MiniMath::V3* normals, ui32 numberOfElements)
{
	this->normals.AddData(normals, numberOfElements);
}

void Renderable::AddColors(const vector<MiniMath::V3>& colors)
{
	this->colors3.AddData(colors.data(), colors.size());
}

void Renderable::AddColors(const MiniMath::V3* colors, ui32 numberOfElements)
{
	this->colors3.AddData(colors, numberOfElements);
}

void Renderable::AddColors(const vector<MiniMath::V4>& colors)
{
	this->colors4.AddData(colors.data(), colors.size());
}

void Renderable::AddColors(const MiniMath::V4* colors, ui32 numberOfElements)
{
	this->colors4.AddData(colors, numberOfElements);
}

void Renderable::AddUVs(const vector<MiniMath::V2>& uvs)
{
	this->uvs.AddData(uvs.data(), uvs.size());
}

void Renderable::AddUVs(const MiniMath::V2* uvs, ui32 numberOfElements)
{
	this->uvs.AddData(uvs, numberOfElements);
}

void Renderable::AddInstanceTransforms(const vector<MiniMath::M4>& transforms)
{
	this->instanceTransforms.AddData(transforms.data(), transforms.size());
}

void Renderable::AddInstanceTransforms(const MiniMath::M4* transforms, ui32 numberOfElements)
{
	this->instanceTransforms.AddData(transforms, numberOfElements);
}
