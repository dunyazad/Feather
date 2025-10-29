#pragma once

#include <FeatherCommon.h>

struct EditorNode
{
	int   id;
	float value;

	EditorNode(const int i, const float v) : id(i), value(v) {}
};

struct NodeLink
{
	int id;
	int start_attr, end_attr;
};

struct Editor
{
	ImNodesEditorContext* context = nullptr;
	std::vector<EditorNode>     nodes;
	std::vector<NodeLink>     links;
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
