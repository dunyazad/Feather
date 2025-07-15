#pragma once

#include <FeatherCommon.h>

class GeometryBuilder
{
public:
	GeometryBuilder();
	~GeometryBuilder();

	static tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
		BuildPlane(f32 width, f32 height, const MiniMath::V3& center, const MiniMath::V3& normal, const MiniMath::V4& color = MiniMath::V4(1.0f, 1.0f, 1.0f, 1.0f));

	static tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
		BuildBox(const MiniMath::V3& center, const MiniMath::V3& dimension, const MiniMath::V4& color = MiniMath::V4(1.0f, 1.0f, 1.0f, 1.0f));

	static tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
		BuildWiredBox(const MiniMath::V3& center, const MiniMath::V3& dimension, const MiniMath::V4& color = MiniMath::V4(1.0f, 1.0f, 1.0f, 1.0f));

	static tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
		BuildSphere(const MiniMath::V3& center, f32 radius, ui32 horizontalSegments, ui32 verticalSegments, const MiniMath::V4& color = MiniMath::V4(1.0f, 1.0f, 1.0f, 1.0f));
};
