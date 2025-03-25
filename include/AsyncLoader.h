#pragma once

#include <FeatherCommon.h>

class Renderable;

class AsyncLoader
{
public:
	AsyncLoader();
	~AsyncLoader();

	void LoadAsync(Renderable* renderable, const string& filename);
};
