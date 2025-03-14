#include <Component/Renderable.h>

Renderable::Renderable()
	: RegisterDerivation<Renderable, ComponentBase>()
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
	colors.Initialize(2, GraphicsBuffer<MiniMath::V4>::BufferTarget::Array);
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
	colors.Update();
	uvs.Update();

	if (numberOfInstances > 1)
	{
		instanceTransforms.Update();
	}

	glBindVertexArray(0);
}

void Renderable::Draw()
{
	glBindVertexArray(vao);

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

void Renderable::AddColor(const MiniMath::V4& color)
{
	colors.AddData(color);
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

void Renderable::AddVertices(const vector<MiniMath::V3>& vertices)
{
	this->vertices.AddData(vertices.data(), vertices.size());
}

void Renderable::AddNormals(const vector<MiniMath::V3>& normals)
{
	this->normals.AddData(normals.data(), normals.size());
}

void Renderable::AddColors(const vector<MiniMath::V4>& colors)
{
	this->colors.AddData(colors.data(), colors.size());
}

void Renderable::AddUVs(const vector<MiniMath::V2>& uvs)
{
	this->uvs.AddData(uvs.data(), uvs.size());
}

void Renderable::AddInstanceTransforms(const vector<MiniMath::M4>& transforms)
{
	this->instanceTransforms.AddData(transforms.data(), transforms.size());
}