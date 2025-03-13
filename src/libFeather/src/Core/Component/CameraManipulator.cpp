#include <Core/Component/CameraManipulator.h>
#include <Feather.h>

CameraManipulator::CameraManipulator(ComponentID id)
	: ComponentBase(id)
{
	AddEventHandler(EventType::MousePosition, [](const Event& event) {
		alog("x : %.2f, y : %.2f\n",
			event.parameters.mousePosition.xpos,
			event.parameters.mousePosition.ypos);
		});

	AddEventHandler(EventType::MouseButtonPress, [](const Event& event) {
		alog("Button Press : %d - x : %.2f, y : %.2f\n",
			event.parameters.mouseButton.button,
			event.parameters.mouseButton.xpos,
			event.parameters.mouseButton.ypos);
		});

	AddEventHandler(EventType::MouseButtonRelease, [](const Event& event) {
		alog("Button Release : %d - x : %.2f, y : %.2f\n",
			event.parameters.mouseButton.button,
			event.parameters.mouseButton.xpos,
			event.parameters.mouseButton.ypos);
		});
}

CameraManipulator::~CameraManipulator()
{
}
