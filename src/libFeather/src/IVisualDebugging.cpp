#include <IVisualDebugging.h>
#include <VisualDebugging.h>

void IVisualDebugging::CreateLineEntity(const std::string& tag)
{
	VisualDebugging::CreateLineEntity(tag);
}

void IVisualDebugging::CreateTriangleEntity(const std::string& tag)
{
	VisualDebugging::CreateTriangleEntity(tag);
}

void IVisualDebugging::CreateBoxEntity(const std::string& tag)
{
	VisualDebugging::CreateBoxEntity(tag);
}

void IVisualDebugging::CreateWiredBoxEntity(const std::string& tag)
{
	VisualDebugging::CreateWiredBoxEntity(tag);
}

void IVisualDebugging::CreateSphereEntity(const std::string& tag)
{
	VisualDebugging::CreateSphereEntity(tag);
}

void IVisualDebugging::CreateTextBlockEntity(const std::string& tag)
{
	VisualDebugging::CreateTextBlockEntity(tag);
}

void IVisualDebugging::Clear(const std::string& tag)
{
	VisualDebugging::Clear(tag);
}

void IVisualDebugging::ClearAll()
{
	VisualDebugging::ClearAll();
}

void IVisualDebugging::SetVisibility(bool visible, const std::string& tag)
{
	VisualDebugging::SetVisibility(visible, tag);
}

void IVisualDebugging::SetVisibilityAll(bool visible)
{
	VisualDebugging::SetVisibilityAll(visible);
}

void IVisualDebugging::ToggleVisibility(const std::string& tag)
{
	VisualDebugging::ToggleVisibility(tag);
}

void IVisualDebugging::ToggleVisibilityAll()
{
	VisualDebugging::ToggleVisibilityAll();
}

void IVisualDebugging::AddLine(const std::string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c)
{
	VisualDebugging::AddLine(tag, v0, v1, c);
}

void IVisualDebugging::AddLine(const std::string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec4& c0, const glm::vec4& c1)
{
	VisualDebugging::AddLine(tag, v0, v1, c0, c1);
}

void IVisualDebugging::AddLine(const std::string& tag, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector4f& c)
{
	VisualDebugging::AddLine(tag, v0, v1, c);
}

void IVisualDebugging::AddLine(const std::string& tag, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector4f& c0, const Eigen::Vector4f& c1)
{
	VisualDebugging::AddLine(tag, v0, v1, c0, c1);
}

void IVisualDebugging::AddTriangle(const std::string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec4& c)
{
	VisualDebugging::AddTriangle(tag, v0, v1, v2, c);
}

void IVisualDebugging::AddTriangle(const std::string& tag, const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2, const glm::vec4& c0, const glm::vec4& c1, const glm::vec4& c2)
{
	VisualDebugging::AddTriangle(tag, v0, v1, v2, c0, c1, c2);
}

void IVisualDebugging::AddTriangle(const std::string& tag, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector4f& c)
{
	VisualDebugging::AddTriangle(tag, v0, v1, v2, c);
}

void IVisualDebugging::AddTriangle(const std::string& tag, const Eigen::Vector3f& v0, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector4f& c0, const Eigen::Vector4f& c1, const Eigen::Vector4f& c2)
{
	VisualDebugging::AddTriangle(tag, v0, v1, v2, c0, c1, c2);
}

void IVisualDebugging::AddBox(const std::string& tag, const AABB& aabb, const glm::vec4& color)
{
	VisualDebugging::AddBox(tag, aabb, color);
}

void IVisualDebugging::AddBox(const std::string& tag, const glm::vec3& center, const glm::vec3& dimensions, const glm::vec4& color)
{
	VisualDebugging::AddBox(tag, center, dimensions, color);
}

void IVisualDebugging::AddBox(const std::string& tag, const glm::vec3& center, const glm::vec3& normal, const glm::vec3& dimensions, const glm::vec4& color)
{
	VisualDebugging::AddBox(tag, center, normal, dimensions, color);
}

void IVisualDebugging::AddBox(const std::string& tag, const Eigen::AABB& aabb, const Eigen::Vector4f& color)
{
	VisualDebugging::AddBox(tag, aabb, color);
}

void IVisualDebugging::AddBox(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& dimensions, const Eigen::Vector4f& color)
{
	VisualDebugging::AddBox(tag, center, dimensions, color);
}

void IVisualDebugging::AddBox(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& normal, const Eigen::Vector3f& dimensions, const Eigen::Vector4f& color)
{
	VisualDebugging::AddBox(tag, center, normal, dimensions, color);
}

void IVisualDebugging::AddWiredBox(const std::string& tag, const AABB& aabb, const glm::vec4& color)
{
	VisualDebugging::AddWiredBox(tag, aabb, color);
}

void IVisualDebugging::AddWiredBox(const std::string& tag, const glm::vec3& center, const glm::vec3& dimensions, const glm::vec4& color)
{
	VisualDebugging::AddWiredBox(tag, center, dimensions, color);
}

void IVisualDebugging::AddWiredBox(const std::string& tag, const glm::vec3& center, const glm::vec3& normal, const glm::vec3& dimensions, const glm::vec4& color)
{
	VisualDebugging::AddWiredBox(tag, center, normal, dimensions, color);
}

void IVisualDebugging::AddWiredBox(const std::string& tag, const Eigen::AABB& aabb, const Eigen::Vector4f& color)
{
	VisualDebugging::AddWiredBox(tag, aabb, color);
}

void IVisualDebugging::AddWiredBox(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& dimensions, const Eigen::Vector4f& color)
{
	VisualDebugging::AddWiredBox(tag, center, dimensions, color);
}

void IVisualDebugging::AddWiredBox(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& normal, const Eigen::Vector3f& dimensions, const Eigen::Vector4f& color)
{
	VisualDebugging::AddWiredBox(tag, center, normal, dimensions, color);
}

void IVisualDebugging::AddSphere(const std::string& tag, const glm::vec3& center, float radius, const glm::vec4& color)
{
	VisualDebugging::AddSphere(tag, center, radius, color);
}

void IVisualDebugging::AddSphere(const std::string& tag, const glm::vec3& center, const glm::vec3& normal, float radius, const glm::vec4& color)
{
	VisualDebugging::AddSphere(tag, center, normal, radius, color);
}

void IVisualDebugging::AddSphere(const std::string& tag, const Eigen::Vector3f& center, float radius, const Eigen::Vector4f& color)
{
	VisualDebugging::AddSphere(tag, center, radius, color);
}

void IVisualDebugging::AddSphere(const std::string& tag, const Eigen::Vector3f& center, const Eigen::Vector3f& normal, float radius, const Eigen::Vector4f& color)
{
	VisualDebugging::AddSphere(tag, center, normal, radius, color);
}

void IVisualDebugging::AddText(const std::string& tag, const std::string& text, const glm::vec3& position, const glm::vec4& color, float fontSize)
{
	VisualDebugging::AddText(tag, text, position, color, fontSize);
}

void IVisualDebugging::AddText(const std::string& tag, const std::string& text, const Eigen::Vector3f& position, const Eigen::Vector4f& color, float fontSize)
{
	VisualDebugging::AddText(tag, text, position, color, fontSize);
}

void IVisualDebugging::ClearSelectionList()
{
	VisualDebugging::ClearSelectionList();
}

void IVisualDebugging::AddToSelectionList(const std::string& tag)
{
	VisualDebugging::AddToSelectionList(tag);
}

unsigned int IVisualDebugging::ShowNextSelection()
{
	return VisualDebugging::ShowNextSelection();
}

unsigned int IVisualDebugging::ShowPreviousSelection()
{
	return VisualDebugging::ShowPreviousSelection();
}
