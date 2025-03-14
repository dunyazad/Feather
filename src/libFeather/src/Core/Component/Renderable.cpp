#include <Core/Component/Renderable.h>

Renderable::Renderable(ComponentID id)
	: ComponentBase(id)
{
}

Renderable::~Renderable()
{
}

void Renderable::Initialize(GeometryMode geometryMode)
{
	this->geometryMode = geometryMode;

	glGenVertexArrays(1, &vao);

	vertices.Initialize(0, Buffer<MiniMath::V3>::BufferTarget::Array);
	normals.Initialize(1, Buffer<MiniMath::V3>::BufferTarget::Array);
	colors.Initialize(2, Buffer<MiniMath::V4>::BufferTarget::Array);
	indices.Initialize(3, Buffer<ui32>::BufferTarget::Element);

	//unsigned int VAO, VBO, EBO, instanceVBO;
	//glGenVertexArrays(1, &VAO);
	//glGenBuffers(1, &VBO);
	//glGenBuffers(1, &EBO);
	//glGenBuffers(1, &instanceVBO);

	//glBindVertexArray(VAO);

	//// Setup vertex attributes
	//glBindBuffer(GL_ARRAY_BUFFER, VBO);
	//glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

	//// Position attribute
	//glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
	//glEnableVertexAttribArray(0);

	//// Texture coordinates attribute
	//glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
	//glEnableVertexAttribArray(1);

	//// Setup instance buffer
	//glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
	//glBufferData(GL_ARRAY_BUFFER, sizeof(instancePositions), instancePositions, GL_STATIC_DRAW);

	//// Instance attribute
	//glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	//glEnableVertexAttribArray(2);
	//glVertexAttribDivisor(2, 1);  // Tell OpenGL this is an instanced attribute

	//glBindVertexArray(0);

}

void Renderable::Update(ui32 frameNo, f32 timeDelta)
{
	if (UINT_MAX == vao)
	{
		Initialize(geometryMode);
	}

	glBindVertexArray(vao);

	vertices.Update();
	normals.Update();
	colors.Update();
	indices.Update();
}

void Renderable::Draw()
{
	glBindVertexArray(vao);

	glDrawElements(geometryMode, indices.size(), GL_UNSIGNED_INT, nullptr);
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

void Renderable::AddIndex(ui32 index)
{
	indices.AddData(index);
}
