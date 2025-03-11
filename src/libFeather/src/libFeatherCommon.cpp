#include <libFeather.h>

#include <Core/System/RenderSystem.h>
#include <Core/System/GUISystem.h>

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

Entity* Feather::CreateEntity()
{
    return s_instance->CreateEntity();
}

Entity* Feather::GetEntity(EntityID id)
{
    return s_instance->GetEntity(id);
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

RenderSystem* Feather::GetRenderSystem()
{
    return dynamic_cast<RenderSystem*>(GetSystem("RenderSystem"));
}

GUISystem* Feather::GetGUISystem()
{
    return dynamic_cast<GUISystem*>(GetSystem("GUISystem"));
}
