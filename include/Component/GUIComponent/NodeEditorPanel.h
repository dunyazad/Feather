#pragma once

#include <FeatherCommon.h>

struct Node
{
	int   id;
	float value;

	Node(const int i, const float v) : id(i), value(v) {}
};

struct Link
{
	int id;
	int start_attr, end_attr;
};

struct Editor
{
	ImNodesEditorContext* context = nullptr;
	std::vector<Node>     nodes;
	std::vector<Link>     links;
	int                   current_id = 0;
};

class NodeEditorPanel
{
public:
	NodeEditorPanel(const std::string& title = "");
	~NodeEditorPanel();

	virtual void Render();
	//private:

	std::string title = "";

private:
	bool initialized = false;
	Editor editor1;
	Editor editor2;
};
