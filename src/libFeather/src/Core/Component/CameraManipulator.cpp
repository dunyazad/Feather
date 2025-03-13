#include <Core/Component/CameraManipulator.h>
#include <Feather.h>

CameraManipulator::CameraManipulator(ComponentID id)
	: EventReceiver(id)
{
	AddEventHandler(EventType::MousePosition, [](const Event& event) {
		alog("x : %f, y : %f\n", event.parameters.mousePosition.xpos, event.parameters.mousePosition.ypos);
		});

	AddEventHandler(EventType::MouseButtonPress, [](const Event& event) {
		alog("Button Press : %d\n", event.parameters.mouseButton.button);
		});

	AddEventHandler(EventType::MouseButtonRelease, [](const Event& event) {
		alog("Button Release : %d\n", event.parameters.mouseButton.button);
		});
}

CameraManipulator::~CameraManipulator()
{
}
