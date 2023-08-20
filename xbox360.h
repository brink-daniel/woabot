
#include <unistd.h>
#include <linux/joystick.h>
#include <math.h>

constexpr float event_offset = 32767;
constexpr float dead_zone = 20;

enum button
{
	A = 0,
	B = 1,
	X = 2,
	Y = 3,
	LeftBumper = 4,
	RightBumper = 5,
	Back = 6,
	Start = 7,
	Left = 11,
	Right = 12,
	Up = 13,
	Down = 14
};

enum axis
{
	LeftStick_X = 0,
	LeftStick_Y = 1,
	LeftTrigger = 2,
	RightStick_X = 3,
	RightStick_Y = 4,
	RightTrigger = 5,
	DPad_X = 6,
	DPad_Y = 7
};

int read_event(int fd, struct js_event *event)
{
	ssize_t bytes;
	bytes = read(fd, event, sizeof(*event));

	if (bytes == sizeof(*event))
	{
		return 0;
	}
	return -1;
}

const int axis_value(axis event_number, int event_value)
{
	switch (event_number)
	{
	case LeftStick_X:
	case LeftStick_Y:
	case RightStick_X:
	case RightStick_Y:
	case DPad_X:
	case DPad_Y:
		return event_value / event_offset * 100;
	case LeftTrigger:
	case RightTrigger:
		return (event_value + event_offset) / (event_offset * 2) * 100;
	default:
		return 0;
	}
}

const int axis_deadzone_filter(axis event_number, int event_value)
{
	switch (event_number)
	{
	case LeftStick_X:
	case LeftStick_Y:
	case RightStick_X:
	case RightStick_Y:
		if (abs(event_value) <= dead_zone)
		{
			return 0;
		}
		break;
	}
	return event_value;
}