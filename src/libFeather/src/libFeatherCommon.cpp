#include <libFeather.h>

#include <Core/System/GUISystem.h>
#include <Core/System/InputSystem.h>
#include <Core/System/ImmediateModeRenderSystem.h>
#include <Core/System/RenderSystem.h>

namespace Time
{
    chrono::steady_clock::time_point Now()
    {
        return chrono::high_resolution_clock::now();
    }

    uint64_t Microseconds(chrono::steady_clock::time_point& from, chrono::steady_clock::time_point& now)
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(now - from).count();
    }

    chrono::steady_clock::time_point End(chrono::steady_clock::time_point& from, const string& message, int number)
    {
        auto now = chrono::high_resolution_clock::now();
        if (-1 == number)
        {
            printf("[%s] %.4f ms from start\n", message.c_str(), (float)(Microseconds(from, now)) / 1000.0f);
        }
        else
        {
            printf("[%6d - %s] %.4f ms from start\n", number, message.c_str(), (float)(Microseconds(from, now)) / 1000.0f);
        }
        return now;
    }

    string DateTime()
    {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S"); // Format: YYYYMMDD_HHMMSS
        return oss.str();
    }
}

Feather::Feather() {}
Feather::~Feather() {}

FeatherWindow* Feather::GetFeatherWindow()
{
    return s_instance->featherWindow;
}

GLFWwindow* Feather::GetGLFWWindow()
{
    return s_instance->featherWindow->GetGLFWwindow();
}

Entity* Feather::CreateEntity(const string& name)
{
    return s_instance->CreateEntity(name);
}

Entity* Feather::GetEntity(EntityID id)
{
    return s_instance->GetEntity(id);
}

PerspectiveCamera* Feather::CreatePerspectiveCamera()
{
    return s_instance->CreatePerspectiveCamera();
}

ComponentBase* Feather::GetComponent(ComponentID id)
{
    return s_instance->GetComponent(id);
}

const vector<ComponentBase*>& Feather::GetComponents()
{
    return s_instance->GetComponents();
}

const vector<ComponentID>& Feather::GetComponentsByEntityID(EntityID entityID)
{
    return s_instance->GetComponentsByEntityID(entityID);
}

const vector<ui32>& Feather::GetComponentIDsByTypeIndex(const type_index& typeIndex)
{
    return s_instance->GetComponentIDsByTypeIndex(typeIndex);
}

SystemBase* Feather::GetSystem(const string& systemName)
{
    if (0 == s_instance->systems.count(systemName))
    {
        return nullptr;
    }
    else
    {
        return s_instance->systems[systemName];
    }
}

GUISystem* Feather::GetGUISystem()
{
    return dynamic_cast<GUISystem*>(GetSystem("GUISystem"));
}

InputSystem* Feather::GetInputSystem()
{
    return dynamic_cast<InputSystem*>(GetSystem("InputSystem"));
}

RenderSystem* Feather::GetRenderSystem()
{
    return dynamic_cast<RenderSystem*>(GetSystem("RenderSystem"));
}

ImmediateModeRenderSystem* Feather::GetImmediateModeRenderSystem()
{
    return dynamic_cast<ImmediateModeRenderSystem*>(GetSystem("ImmediateModeRenderSystem"));
}
