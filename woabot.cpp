#include <fcntl.h>
#include <phidget22.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include <fstream>
#include <iomanip>
#include "xbox360.h"

bool terminate = false;
bool shutdown = false;

constexpr int max_power = 100;
constexpr int high_power = 80;

double max_acceleration = 0;
double low_acceleration = 0;

int left_power = max_power;
int right_power = max_power;
int power_calibrate = 0;
int left_power_reduce_by = 0;
int right_power_reduce_by = 0;
int forward_power = 0;
int reverse_power = 0;
int throttle = 0;

bool boost = false;

double current_acceleration = 0;
double current_velocity_left = 0;
double current_velocity_right = 0;

std::ofstream log_file("woabot.log");

PhidgetDCMotorHandle motor_left;
PhidgetDCMotorHandle motor_right;

void Reset()
{
	left_power = max_power;
	right_power = max_power;
	power_calibrate = 0;
	left_power_reduce_by = 0;
	right_power_reduce_by = 0;
	forward_power = 0;
	reverse_power = 0;
	throttle = 0;
	boost = false;
	current_acceleration = 0;
	current_velocity_left = 0;
	current_velocity_right = 0;
}

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

void Log(std::string info)
{
	Log(info, false);
}

void Error(std::string msg)
{
	Log(msg, true);
}

void SignalHandler(int s)
{
	Log("Caught signal. Terminating...");
	terminate = true;
}

void Shutdown()
{
	Log("Shutdown triggered");
	terminate = true;
	shutdown = true;
}

void PowerBalanceCalibrate(signed short value)
{
	if (abs(value) != max_power)
	{
		return;
	}

	if (value > 0)
	{
		// steer more right
		power_calibrate += 1;

		if (power_calibrate > 10)
		{
			power_calibrate = 10;
		}
	}
	else
	{
		// steer more left
		power_calibrate -= 1;

		if (power_calibrate < -10)
		{
			power_calibrate = -10;
		}
	}

	if (power_calibrate > 0)
	{
		// steer more right
		left_power_reduce_by = 0;
		right_power_reduce_by = power_calibrate;
	}
	else if (power_calibrate < 0)
	{
		// steer more left
		left_power_reduce_by = abs(power_calibrate);
		right_power_reduce_by = 0;
	}
	else
	{
		left_power_reduce_by = 0;
		right_power_reduce_by = 0;
	}
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
		// turn right
		left_power = max_power;
		right_power = max_power - value;
	}
	else
	{
		// turn left
		left_power = max_power - abs(value);
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
		Error("Invalid velocity");
		return 1;
	}
	if (value < -1)
	{
		Error("Invalid velocity");
		return -1;
	}
	return value;
}

double ApplyBoost(double velocity)
{
	if (!boost || velocity == 0)
	{
		return velocity;
	}

	if (velocity > 0)
	{
		if (velocity < max_power)
		{
			return 1;
		}
	}
	else
	{
		if (velocity > -max_power)
		{
			return -1;
		}
	} 
	return velocity;
}


bool SetVelocity()
{
	bool success = true;

	double d_throttle = throttle;
	double d_max_power = max_power;
	double d_left_power = left_power;
	double d_right_power = right_power;
	double d_left_power_reduce_by = left_power_reduce_by;
	double d_right_power_reduce_by = right_power_reduce_by;


	double left_velocity = (d_throttle / d_max_power) * (d_left_power / d_max_power);
	left_velocity = MinMaxVelocity(left_velocity);
	if (left_velocity > 0)
	{
		left_velocity -= (d_left_power_reduce_by / d_max_power);
	}
	else if (left_velocity < 0)
	{
		left_velocity += (d_left_power_reduce_by / d_max_power);
	}

	left_velocity = ApplyBoost(left_velocity);
	
	if (current_velocity_left != left_velocity)
	{
		auto res1 = PhidgetDCMotor_setTargetVelocity(motor_left, left_velocity);
		if (res1 != EPHIDGET_OK)
		{
			Error("Left target velocity not set");
			success = false;
		}
		else 
		{
			current_velocity_left = left_velocity;
		}
	}

	double right_velocity = (d_throttle / d_max_power) * (d_right_power / d_max_power);
	right_velocity = MinMaxVelocity(right_velocity);
	if (right_velocity > 0)
	{
		right_velocity -= (d_right_power_reduce_by / d_max_power);
	}
	else if (right_velocity < 0)
	{
		right_velocity += (d_right_power_reduce_by / d_max_power);
	}

	right_velocity = ApplyBoost(right_velocity);

	if (current_velocity_right != right_velocity)
	{
		auto res2 = PhidgetDCMotor_setTargetVelocity(motor_right, right_velocity);
		if (res2 != EPHIDGET_OK)
		{
			Error("Right target velocity not set");
			success = false;
		}
		else 
		{
			current_velocity_right = right_velocity;
		}
	}

	return success;
}

bool SetAcceleration(double value)
{
	if (current_acceleration == value)
	{
		return true;
	}

	auto res1 = PhidgetDCMotor_setAcceleration(motor_left, value);
	auto res2 = PhidgetDCMotor_setAcceleration(motor_right, value);

	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		return false;
	}

	current_acceleration = value;
	return true;
}

void Boost(signed short event_value)
{
	if (event_value > 0)
	{
		SetAcceleration(max_acceleration);
		boost = true;
		return;
	}
	SetAcceleration(low_acceleration);
	boost = false;
}

void Afterburner(signed short event_value)
{
	PowerBalance(0);
	ThrottleReverse(0);
	if (event_value > 0)
	{
		SetAcceleration(max_acceleration);
		ThrottleForward(max_power);
	}
	else
	{
		SetAcceleration(low_acceleration);
		ThrottleForward(0);
	}
}


void DriveForward(signed short event_value)
{
	PowerBalance(0);
	ThrottleReverse(0);
		
	if (event_value > 0)
	{
		ThrottleForward(high_power);
	}
	else
	{
		ThrottleForward(0);
	}
}

void DriveBackwards(signed short event_value)
{
	PowerBalance(0);
	ThrottleForward(0);

	if (event_value > 0)
	{
		ThrottleReverse(high_power);
	}
	else
	{
		ThrottleReverse(0);
	}
}

void TurnLeft(signed short event_value)
{
	ThrottleReverse(0);
	
	if (event_value <= 0)
	{
		left_power = max_power;
		right_power = max_power;
		ThrottleForward(0);
		return;
	}
	
	left_power = -max_power;
	right_power = max_power;
	ThrottleForward(high_power);
}

void TurnRight(signed short event_value)
{
	ThrottleReverse(0);
	
	if (event_value<=0)
	{
		PowerBalance(0);
		ThrottleForward(0);
		return;
	}
	
	left_power = max_power;
	right_power = -max_power;
	ThrottleForward(high_power);
}

void ProcessButtonEvent(unsigned char event_number, signed short event_value)
{
	auto a = static_cast<button>(event_number);
	switch (a)
	{
	case A:
		Afterburner(event_value);
		break;
	case B:
		Boost(event_value);
		break;
	case X:
		break;
	case Y:
		break;
	case LeftBumper:
		break;
	case RightBumper:
		break;
	case Back:
		Shutdown();
		break;
	case Start:
		Reset();
		break;
	case Left:
		TurnLeft(event_value);
		break;
	case Right:
		TurnRight(event_value);
		break;
	case Up:
		DriveForward(event_value);
		break;
	case Down:
		DriveBackwards(event_value);
		break;
	}
	SetVelocity();
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
		PowerBalanceCalibrate(p);
		break;
	case RightStick_Y:
		break;
	case DPad_X:
		break;
	case DPad_Y:
		break;
	}
	SetVelocity();
}

int main(int argc, char *argv[])
{
	Log("Woabot started");

	// catch ctrl+c
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = SignalHandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	Log("Pausing 5 seconds for OS hardware detection...");
	using namespace std::chrono_literals;
	std::this_thread::sleep_for(5s);

	// init HC MotorController
	PhidgetDCMotor_create(&motor_left);
	PhidgetDCMotor_create(&motor_right);
	Phidget_setChannel((PhidgetHandle)motor_left, 0);
	Phidget_setChannel((PhidgetHandle)motor_right, 1);
	PhidgetReturnCode res1;
	PhidgetReturnCode res2;

	do
	{
		res1 = Phidget_openWaitForAttachment((PhidgetHandle)motor_left, 5000);
		if (res1 != EPHIDGET_OK)
		{
			Error("Left motor controller not connected");
			std::this_thread::sleep_for(1s);			
		}
	} while (!terminate && res1 != EPHIDGET_OK);
	Log("Left motor controller connected");

	do
	{
		res2 = Phidget_openWaitForAttachment((PhidgetHandle)motor_right, 5000);
		if (res2 != EPHIDGET_OK)
		{
			Error("Right motor controller not connected");
			std::this_thread::sleep_for(1s);
		}
	} while (!terminate && res2 != EPHIDGET_OK);
	Log("Right motor controller connected");

	
	do
	{
		res1 = PhidgetDCMotor_getMaxAcceleration(motor_left, &max_acceleration);
		if (res1 != EPHIDGET_OK)
		{
			Error("Max acceleration unknown");
			std::this_thread::sleep_for(1s);
		}
	} while (!terminate && res1 != EPHIDGET_OK);
	Log("Max acceleration set");
	low_acceleration = max_acceleration / 3;

	while (!SetAcceleration(low_acceleration) && !terminate)
	{
		Error("Default acceleration not set");
		std::this_thread::sleep_for(1s);
	}
	Log("Default acceleration set");


	// init xbox360 controller
	int js = -1;
	struct js_event event;
	do
	{
		js = open("/dev/input/js0", O_RDONLY);
		if (js == -1)
		{
			Error("XBox360 controller not connected");
			std::this_thread::sleep_for(1s);
		}
	} while (!terminate && js == -1);
	Log("XBox360 controller connected");

	// Control loop

	Log("Listening for controller inputs...");
	while (read_event(js, &event) == 0 && !terminate)
	{
		switch (event.type)
		{
		case JS_EVENT_AXIS:
			ProcessAxisEvent(event.number, event.value);
			break;
		case JS_EVENT_BUTTON:
			ProcessButtonEvent(event.number, event.value);
			break;
		default:
			break;
		}
	}

	// Shutdown start
	Error("Stopped listening for controller inputs");

	res1 = PhidgetDCMotor_setTargetVelocity(motor_left, 0);
	res2 = PhidgetDCMotor_setTargetVelocity(motor_right, 0);
	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		Error("Velocity not set to 0");
		return 1;
	}
	Log("Velocity set to 0");

	res1 = Phidget_close((PhidgetHandle)motor_left);
	res2 = Phidget_close((PhidgetHandle)motor_right);
	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		Error("Motor connection not closed");
		return 1;
	}
	Log("Motor connection closed");

	res1 = PhidgetDCMotor_delete(&motor_left);
	res2 = PhidgetDCMotor_delete(&motor_right);
	if (res1 != EPHIDGET_OK || res2 != EPHIDGET_OK)
	{
		Error("Motor connection not deleted");
		return 1;
	}
	Log("Motor connection deleted");

	close(js);

	Log("End");
	log_file.close();

	if (shutdown)
	{
		system("shutdown -P now");
	}

	return 0;
}