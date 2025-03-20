#include <libFeather.h>

#include <System/GUISystem.h>
#include <System/InputSystem.h>
#include <System/ImmediateModeRenderSystem.h>
#include <System/RenderSystem.h>

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

//unordered_map<type_index, unordered_set<type_index>> FeatherObject::subclass_map;
//
//unordered_map<type_index, unordered_set<type_index>>& FeatherObject::GetSubclassMap()
//{
//	return subclass_map;
//}
//
//void FeatherObject::RegisterClass(type_index baseType, type_index derivedType)
//{
//	GetSubclassMap()[baseType].insert(derivedType);
//}
//
//unordered_set<type_index> FeatherObject::GetAllSubclasses(type_index baseType)
//{
//	unordered_set<type_index> subclasses;
//	if (GetSubclassMap().find(baseType) != GetSubclassMap().end())
//	{
//		for (const auto& subType : GetSubclassMap()[baseType])
//		{
//			if (subclasses.insert(subType).second)
//			{
//				auto childSubclasses = GetAllSubclasses(subType);
//				subclasses.insert(childSubclasses.begin(), childSubclasses.end());
//			}
//		}
//	}
//	return subclasses;
//}
//
//void FeatherObject::OnEvent(const Event& event)
//{
//    if (0 != eventHandlers.count(event.type))
//    {
//        for (auto& handler : eventHandlers[event.type])
//        {
//            handler(event, this);
//        }
//    }
//}
//
//void FeatherObject::SubscribeEvent(EventType eventType)
//{
//    auto eventSystems = Feather.GetInstances<EventSystem>();
//    if (eventSystems.empty()) return;
//
//    if (nullptr != *eventSystems.begin())
//    {
//        (*eventSystems.begin())->SubscribeEvent(eventType, this);
//    }
//}
//
//void FeatherObject::AddEventHandler(EventType eventType, function<void(const Event&, FeatherObject*)> handler)
//{
//    eventHandlers[eventType].push_back(handler);
//    SubscribeEvent(eventType);
//}