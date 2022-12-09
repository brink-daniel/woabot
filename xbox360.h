
#include <unistd.h>
#include <linux/joystick.h>
#include <math.h>

constexpr float event_offset = 32767;
constexpr float dead_zone = 20;

enum button
{
	// not used
	A = 0,
	// not used
	B = 1,
	// not used
	X = 2,
	// not used
	Y = 3,
	// not used
	LeftBumper = 4,
	// not used
	RightBumper = 5,
	// not used
	Back = 6,
	// not used
	Start = 7,
	// not used
	Left = 11,
	// not used
	Right = 12,
	// not used
	Up = 13,
	// not used
	Down = 14
};

enum axis
{
	// power balance left vs right
	LeftStick_X = 0,
	// not used
	LeftStick_Y = 1,
	// reverse power
	LeftTrigger = 2,
	// not used
	RightStick_X = 3,
	// not used
	RightStick_Y = 4,
	// forward power
	RightTrigger = 5,
	// not used (also fires on button Left/Right)
	DPad_X = 6,
	// not used (also fires on button Up/Down)
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
		// else if (event_value > dead_zone)
		//{
		//	return (event_value - dead_zone) / (100 - dead_zone) * 100;
		// }
		// else if (event_value < -dead_zone)
		//{
		//	return (event_value + dead_zone) / (100 - dead_zone) * 100;
		// }
		break;
	}
	return event_value;
}