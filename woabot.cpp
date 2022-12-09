#include <fcntl.h>
#include <phidget22.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <fstream>
#include <iomanip>
#include "xbox360.h"

constexpr int max_power = 100;
int left_power = max_power;
int right_power = max_power;
int forward_power = 0;
int reverse_power = 0;
int throttle = 0;
bool terminate = false;

std::ofstream log_file("woabot.log");

PhidgetDCMotorHandle motor_left;
PhidgetDCMotorHandle motor_right;

void Log(std::string info, bool error)
{
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::ostringstream oss;
	std::string err = ": ";
	if (error)
	{
		err = ": ERROR - ";
	}
	oss << std::put_time(&tm, "%Y-%m-%d %H-%M-%S") << err << info << std::endl;	
	auto line = oss.str();
	log_file << line;
	std::cout << line;
}

void SignalHandler(int s)
{
	Log("Caught signal. Terminating...", true);
	terminate = true;
}

void PowerBalance(signed short value)
{
	if (value == 0)
	{
		left_power = max_power;
		right_power = max_power;
		return;
	}

	if (value > 0)
	{
		left_power = max_power;
		if (value >= max_power)
		{
			right_power = 0 - max_power;
		}
		else
		{
			right_power = max_power - abs(value);
		}
	}
	else
	{
		if (abs(value) >= max_power)
		{
			left_power = 0 - max_power;
		}
		else
		{
			left_power = max_power - abs(value);
		}
		right_power = max_power;
	}
}

void UpdateThrottle()
{
	if (!terminate)
	{
		throttle = forward_power - reverse_power;
		return;
	}
	throttle = 0;
}

void ThrottleForward(signed short value)
{
	forward_power = value;
	UpdateThrottle();
}

void ThrottleReverse(signed short value)
{
	reverse_power = value;
	UpdateThrottle();
}

double MinMaxVelocity(double value)
{
	if (value > 1)
	{
		Log("Invalid velocity", true);
		return 1;
	}
	if (value < -1)
	{
		Log("Invalid velocity", true);
		return -1;
	}
	return value;
}

void SignalMotor()
{
	double left_velocity = (throttle / max_power) * (left_power / max_power);
	left_velocity = MinMaxVelocity(left_velocity);
	auto res1 = PhidgetDCMotor_setTargetVelocity(motor_left, left_velocity);
	if (res1 != EPHIDGET_OK)
	{
		Log("Left target velocity not set", true);
	}

	double right_velocity = (throttle / max_power) * (right_power / max_power);
	right_velocity = MinMaxVelocity(right_velocity);
	auto res2 = PhidgetDCMotor_setTargetVelocity(motor_right, right_velocity);
	if (res2 != EPHIDGET_OK)
	{
		Log("Right target velocity not set", true);
	}
}

void ProcessAxisEvent(unsigned char event_number, signed short event_value)
{
	auto a = static_cast<axis>(event_number);
	auto v = axis_value(a, event_value);
	auto p = axis_deadzone_filter(a, v);

	switch (a)
	{
	case LeftStick_X:
		PowerBalance(p);
		break;
	case LeftStick_Y:
		break;
	case RightTrigger:
		ThrottleForward(p);
		break;
	case LeftTrigger:
		ThrottleReverse(p);
		break;
	case RightStick_X:
		break;
	case RightStick_Y:
		break;
	case DPad_X:
		break;
	case DPad_Y:
		break;
	}
	SignalMotor();
}

int main(int argc, char *argv[])
{
	Log("Woabot started", false);

	// catch ctrl+c
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = SignalHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	Log("Pausing 10 seconds for OS hardware detection...", false);
	using namespace std::chrono_literals;
	std::this_thread::sleep_for(10s);

	// init HC MotorController
	PhidgetDCMotor_create(&motor_left);
	PhidgetDCMotor_create(&motor_right);
	Phidget_setChannel((PhidgetHandle)motor_left, 0);
	Phidget_setChannel((PhidgetHandle)motor_right, 1);
	auto res1 = Phidget_openWaitForAttachment((PhidgetHandle)motor_left, 5000);
	auto res2 = Phidget_openWaitForAttachment((PhidgetHandle)motor_right, 5000);

	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		Log("Motor controller not connected", true);
		return 1;
	}
	Log("Motor controller connected", false);

	res1 = PhidgetDCMotor_setAcceleration(motor_left, 1);
	res2 = PhidgetDCMotor_setAcceleration(motor_right, 1);
	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		Log("Default acceleration not set", true);
		return 1;
	}
	Log("Default acceleration set", false);



	// init xbox360 controller
	int js;
	struct js_event event;
	js = open("/dev/input/js0", O_RDONLY);

	if (js == -1)
	{
		Log("XBox360 controller not connected", true);
		return 1;
	}
	Log("XBox360 controller connected", false);

	Log("Listening for controller inputs...", false);
	while (read_event(js, &event) == 0 && !terminate)
	{
		switch (event.type)
		{
		case JS_EVENT_AXIS:
			ProcessAxisEvent(event.number, event.value);
			break;
		default:
			break;
		}
	}

	res1 = PhidgetDCMotor_setTargetVelocity(motor_left, 0);
	res2 = PhidgetDCMotor_setTargetVelocity(motor_right, 0);
	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		Log("Velocity not set to 0", true);
		return 1;
	}
	Log("Velocity set to 0", false);

	res1 = Phidget_close((PhidgetHandle)motor_left);
	res2 = Phidget_close((PhidgetHandle)motor_right);
	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		Log("Motor connection not closed", true);
		return 1;
	}
	Log("Motor connection closed", false);

	res1 = PhidgetDCMotor_delete(&motor_left);
	res2 = PhidgetDCMotor_delete(&motor_right);
	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		Log("Motor connection not deleted", true);
		return 1;
	}
	Log("Motor connection deleted", false);

	close(js);

	Log("End", false);
	log_file.close();

	return 0;
}