#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <string>
#include <phidget22.h>
#include <linux/reboot.h>
#include <iostream>

enum button
{
	// afterburner
	A = 0,
	// boost
	B = 1,
	// decrease acceleration
	X = 2,
	// increase acceleration
	Y = 3,
	// left brake
	LB = 4,
	// right brake
	RB = 5,
	// back
	Back = 6,
	// start
	Start = 7,
	// spin left
	Left = 11,
	// spin right
	Right = 12,
	// forward
	Up = 13,
	// reverse
	Down = 14
};

enum axis
{
	// power balance left vs right
	LS_X = 0,
	// power balance forward vs reverse
	LS_Y = 1,
	// reverse power
	LT = 2,
	// calibrate power balance left vs right
	RS_X = 3,
	// calibrate power balance forward vs reverse
	RS_Y = 4,
	// forward power
	RT = 5,
	// not used (fires on button Left/Right)
	D_X = 6,
	// not used (fires on button Up/Down)
	D_Y = 7
};

constexpr float event_offset = 32767;
constexpr float dead_zone = 10;

int left_power = 100;
int right_power = 100;

int left_power_calibrate = 0;
int right_power_calibrate = 0;

int throttle_forward = 0;
int throttle_reverse = 0;
int throttle = 0;

// PhidgetDCMotorHandle motor_left;
// PhidgetDCMotorHandle motor_right;

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

const int axis_value(int event_number, int event_value)
{
	switch (event_number)
	{
	case 0: // LS X
	case 1: // LS Y
	case 3: // RS X
	case 4: // RS Y
	case 6: // D X
	case 7: // D Y
		return event_value / event_offset * 100;
	case 2: // LT
	case 5: // RT
		return (event_value + event_offset) / (event_offset * 2) * 100;
	default:
		return 0;
	}
}

const int axis_deadzone(int event_number, int event_value)
{
	switch (event_number)
	{
	case 0: // LS X
	case 1: // LS Y
	case 3: // RS X
	case 4: // RS Y
		if (abs(event_value) <= dead_zone)
		{
			return 0;
		}
		else if (event_value > dead_zone)
		{
			return (event_value - dead_zone) / (100 - dead_zone) * 100;
		}
		else if (event_value < -dead_zone)
		{
			return (event_value + dead_zone) / (100 - dead_zone) * 100;
		}
		break;
	}
	return event_value;
}

void Afterburner(signed short value)
{
	std::cout << "afterburner" << '\n';
}

void Boost(signed short value)
{
	std::cout << "boost" << '\n';
}

void SpinToLeft(signed short value)
{
	if (value == 1)
	{
		left_power = -100;
		right_power = 100;
	}
	else
	{
		left_power = 100;
		right_power = 100;
	}
	std::cout << "Spin Left" << left_power << " Right" << right_power << '\n';
}

void SpinToRight(signed short value)
{
	if (value == 1)
	{
		left_power = 100;
		right_power = -100;
	}
	else
	{
		left_power = 100;
		right_power = 100;
	}
	std::cout << "Spin Left" << left_power << " Right" << right_power << '\n';
}

void PowerBalance(signed short value)
{
	if (value > 0)
	{
		left_power = 100 + abs(value);
		right_power = 100 - abs(value);
	}
	else
	{
		left_power = 100 - abs(value);
		right_power = 100 + abs(value);
	}

	std::cout << "Left" << left_power << " Right" << right_power << '\n';
}

void CalibratePowerBalance(signed short value)
{
	if (value == 100 || value == -100)
	{
		if (value == -100)
		{
			right_power_calibrate += 1;
			left_power_calibrate -= 1;
		}
		else if (value == 100)
		{
			right_power_calibrate -= 1;
			left_power_calibrate += 1;
		}
		std::cout << "CalibratePowerBalance "
				  << "Left" << left_power_calibrate << " Right" << right_power_calibrate << '\n';
	}
}

void ResetPowerBalanceCalibration()
{
	right_power_calibrate = 0;
	left_power_calibrate = 0;
	std::cout << "ResetPowerBalanceCalibration "
			  << "Left" << left_power_calibrate << " Right" << right_power_calibrate << '\n';
}

void DecreaseAccl(signed short value)
{
	std::cout << "decrease accelleration" << '\n';
}

void IncreaseAccl(signed short value)
{
	std::cout << "increase accelleration" << '\n';
}

void ResetAccl()
{
	std::cout << "reset accelleration" << '\n';
}

void ThrottleForward(signed short value)
{
	throttle_forward = value;
	throttle = throttle_forward - throttle_reverse;

	std::cout << "forward"<<  throttle_forward  << " throttle" <<  throttle << '\n';
}

void ThrottleReverse(signed short value)
{
	throttle_reverse = value;
	throttle = throttle_forward - throttle_reverse;

	std::cout << "reverse"<<  throttle_reverse  << " throttle" <<  throttle << '\n';
}

void ProcessButtonEvent(unsigned char event_number, signed short event_value)
{
	auto b = static_cast<button>(event_number);
	switch (b)
	{
	case A:
		Afterburner(event_value);
		break;
	case B:
		Boost(event_value);
		break;
	case X:
		DecreaseAccl(event_value);
		break;
	case Y:
		IncreaseAccl(event_value);
		break;
	case LB:
		break;
	case RB:
		break;
	case Left:
		SpinToLeft(event_value);
		break;
	case Right:
		SpinToRight(event_value);
		break;
	case Up:
		break;
	case Down:
		break;
	case Back:
		ResetPowerBalanceCalibration();
		break;
	case Start:
		ResetAccl();
		break;
	}
}

void ProcessAxisEvent(unsigned char event_number, signed short event_value)
{
	auto a = static_cast<axis>(event_number);
	auto v = axis_value(event_number, event_value);
	auto p = axis_deadzone(event_number, v);
	switch (a)
	{
	case LS_X:
		PowerBalance(p);
		break;
	case LS_Y:
		break;
	case RT:
		ThrottleForward(p);
		break;
	case LT:
		ThrottleReverse(p);
		break;
	case RS_X:
		CalibratePowerBalance(p);
	case RS_Y:
		break;
	case D_X:
		break;
	case D_Y:
		break;
	}
}



int main(int argc, char *argv[])
{
	// init xbox360 controller
	int js;
	struct js_event event;
	js = open("/dev/input/js0", O_RDONLY);

	if (js == -1)
	{
		perror("Could not open joystick");
		return 1;
	}

	// init HC MotorController
	// PhidgetDCMotor_create(&motor_left);
	// PhidgetDCMotor_create(&motor_right);
	// Phidget_setChannel((PhidgetHandle)motor_left, 0);
	// Phidget_setChannel((PhidgetHandle)motor_right, 1);
	// auto res1 = Phidget_openWaitForAttachment((PhidgetHandle)motor_left, 5000);
	// auto res2 = Phidget_openWaitForAttachment((PhidgetHandle)motor_right, 5000);

	// if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK )
	//{
	//	perror("Motorcontroller not connected");
	//	return 1;
	// }

	// listen for controller inputs
	while (read_event(js, &event) == 0)
	{
		switch (event.type)
		{
		case JS_EVENT_BUTTON:
			ProcessButtonEvent(event.number, event.value);
			break;
		case JS_EVENT_AXIS:
			ProcessAxisEvent(event.number, event.value);
			break;
		default:
			break;
		}
	}

	close(js);

	// Phidget_close((PhidgetHandle)motor_left);
	// Phidget_close((PhidgetHandle)motor_right);

	// PhidgetDCMotor_delete(&motor_left);
	// PhidgetDCMotor_delete(&motor_right);

	return 0;
}