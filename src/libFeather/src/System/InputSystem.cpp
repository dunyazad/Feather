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

    {
        auto entities = registry.view<Joystick>();
        for (auto& entity : entities)
        {
            auto& joystick = entities.get<Joystick>(entity);
            ControllerInputState state;
            if (joystick.ReadJoystick(state))
            {
				printf("AxisX: %f, AxisY: %f, AxisZ: %f, RotX: %f, RotY: %f, RotZ :%f\n", state.AxisX, state.AxisY, state.AxisZ, state.RotX, state.RotY, state.RotZ);
            }
        }
	}
}
