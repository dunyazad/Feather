#include <libFeatherCommon.h>

#include <Core/Component/ComponentBase.h>

struct ProjectionInfoOrthogonal
{
	f32 left, right, bottom, top, zNear, zFar;
};

struct ProjectionInfoPerspective
{
	f32 fovy, aspectRatio, zNear, zFar;
};

class CameraBase : public ComponentBase
{
public:
	enum ProjectionMode { Perspective, Orghogonal };

public:
	CameraBase(ComponentID id);
	virtual ~CameraBase() = 0;

	MiniMath::V3 position;
	MiniMath::V3 target;

	ProjectionMode projectionMode = Perspective;
protected:
};

class PerspectiveCamera : public CameraBase
{
public:
	PerspectiveCamera(ComponentID id);
	virtual ~PerspectiveCamera();

protected:
	f32 fovy = 45 * DEG2RAD;
	f32 aspectRatio = 1.0f;
	f32 zNear = 0.1f;
	f32 zFar = 1000.0f;
};

class OrthogonalCamera : public CameraBase
{
public:
	OrthogonalCamera(ComponentID id);
	virtual ~OrthogonalCamera();

protected:
	f32 left = -1.0f;
	f32 right = 1.0f;
	f32 bottom = -1.0f;
	f32 top = 1.0f;
	f32 zNear = 0.1f;
	f32 zFar = 1000.0f;
};
