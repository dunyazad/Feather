#pragma once

#include <FeatherCommon.h>

class ComponentBase;

class Entity
{
public:
	Entity(EntityID id, const string& name = "");
	~Entity();

	inline EntityID GetID() const { return id; }
	inline const string& GetName() const { return name; }

	inline const vector<ComponentID>& GetComponentIDs() { return componentIDs; }

	void AddComponent(ComponentBase* component);
	void AddComponent(ComponentID id);

	template<typename T>
	T* GetComponent(int index)
	{
		return dynamic_cast<T*>(Feather::GetInstance().GetComponent(componentIDs[index]));
	}

private:
	string name = "";
	EntityID id;
	vector<ComponentID> componentIDs;
};
