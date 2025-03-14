#pragma once

#include <FeatherCommon.h>

class GeometryBuilder
{
public:
	GeometryBuilder();
	~GeometryBuilder();

	static tuple<vector<ui32>, vector<MiniMath::V3>, vector<MiniMath::V3>, vector<MiniMath::V4>, vector<MiniMath::V2>>
		BuildBox(const MiniMath::V3& center, const MiniMath::V3& dimension);
};
