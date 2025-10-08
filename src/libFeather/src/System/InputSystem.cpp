#include <System/InputSystem.h>
#include <Feather.h>
#include <Component/Components.h>
#include <Component/InputComponent/InputComponents.h>

InputSystem::InputSystem(FeatherWindow* window)
    : window(window)
{
}

InputSystem::~InputSystem()
{
}

void InputSystem::Initialize()
{
}

void InputSystem::Terminate()
{
}

void InputSystem::Update(ui32 frameNo, f32 timeDelta)
{
    auto& registry = Feather.GetRegistry();

    auto entities = registry.view<Joystick>();
    for (auto& entity : entities)
    {
        auto& joystick = entities.get<Joystick>(entity);
        JoystickEvent joystickEvent;
        if (joystick.ReadJoystick(joystickEvent))
        {
            auto& dispatcher = Feather.GetDispatcher();
            dispatcher.enqueue<JoystickEvent>(joystickEvent);
        }
    }
}
