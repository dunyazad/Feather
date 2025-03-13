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

IEventReceiver::IEventReceiver()
{
}

IEventReceiver::~IEventReceiver()
{
}

void IEventReceiver::OnEvent(const Event& event)
{
    if (0 != eventHandlers.count(event.type))
    {
        for (auto& handler : eventHandlers[event.type])
        {
            handler(event);
        }
    }
}

void IEventReceiver::AddEventHandler(EventType eventType, function<void(const Event&)> handler)
{
    eventHandlers[eventType].push_back(handler);
    auto eventSystem = Feather.GetSystem<EventSystem>();
    if (nullptr != eventSystem)
    {
        eventSystem->SubscribeEvent(eventType, this);
    }
}
