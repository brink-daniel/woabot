#include <fcntl.h>
#include <phidget22.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "xbox360.h"

using namespace std::chrono_literals;

int left_power = 100;
int right_power = 100;
int forward_power = 0;
int reverse_power = 0;

double current_velocity_left = 0;
double current_velocity_right = 0;

PhidgetDCMotorHandle motor_left;
PhidgetDCMotorHandle motor_right;

void PowerBalance(signed short value)
{
	if (value == 0)
	{
		left_power = 100;
		right_power = 100;
		return;
	}

	if (value > 0)
	{
		// turn right
		left_power = 100;
		right_power = 100 - value;
	}
	else
	{
		// turn left
		left_power = 100 - abs(value);
		right_power = 100;
	}
}

void ThrottleForward(signed short value)
{
	forward_power = value;
}

void ThrottleReverse(signed short value)
{
	reverse_power = value;
}

double MinMaxVelocity(double value)
{
	if (value > 1)
	{
		return 1;
	}
	if (value < -1)
	{
		return -1;
	}
	return value;
}

void SetVelocity()
{	
	double d_throttle = forward_power - reverse_power;
	double d_max_power = 100;
	double d_left_power = left_power;
	double d_right_power = right_power;	

	double left_velocity = (d_throttle / d_max_power) * (d_left_power / d_max_power);
	left_velocity = MinMaxVelocity(left_velocity);		
	if (current_velocity_left != left_velocity)
	{
		current_velocity_left = left_velocity;
		try 
		{
			PhidgetReturnCode res1 = PhidgetDCMotor_setTargetVelocity(motor_left, left_velocity);
			if (res1 != EPHIDGET_OK)
			{
				std::cout << "Failed to set left motor velocity" << std::endl;
			}
		}
		catch (...)
		{
			std::cout << "Exception setting left motor velocity" << std::endl;
		}
	}

	double right_velocity = (d_throttle / d_max_power) * (d_right_power / d_max_power);
	right_velocity = MinMaxVelocity(right_velocity);	
	if (current_velocity_right != right_velocity)
	{
		current_velocity_right = right_velocity;
		try 
		{
			PhidgetReturnCode res2 = PhidgetDCMotor_setTargetVelocity(motor_right, right_velocity);
			if (res2 != EPHIDGET_OK)
			{
				std::cout << "Failed to set right motor velocity" << std::endl;
			}
		}
		catch (...)
		{
			std::cout << "Exception setting right motor velocity" << std::endl;
		}
	}

	std::cout << "Left: " << current_velocity_left << " Right: " << current_velocity_right << std::endl;
}

void ProcessButtonEvent(unsigned char event_number, signed short event_value)
{
	auto a = static_cast<button>(event_number);
	switch (a)
	{
	case A:
		break;
	case B:
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
		break;
	case Start:
		break;
	case Left:
		break;
	case Right:
		break;
	case Up:
		break;
	case Down:
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
	std::cout << "Woabot" << std::endl;

	
	int js = -1;	
	do
	{
		js = open("/dev/input/js0", O_RDONLY);
		if (js == -1)
		{			
			std::cout << "Waiting for controller..." << std::endl;
			std::this_thread::sleep_for(1s);
		}
	} while (js == -1);



	PhidgetDCMotor_create(&motor_left);
	PhidgetDCMotor_create(&motor_right);
	Phidget_setChannel((PhidgetHandle)motor_left, 0);
	Phidget_setChannel((PhidgetHandle)motor_right, 1);	
	
	PhidgetReturnCode res1;
	do
	{
		res1 = Phidget_openWaitForAttachment((PhidgetHandle)motor_left, 5000);
		if (res1 != EPHIDGET_OK)
		{		
			std::cout << "Waiting for left motor..." << std::endl;	
			std::this_thread::sleep_for(1s);			
		}
	} while (res1 != EPHIDGET_OK);
	

	PhidgetReturnCode res2;
	do
	{
		res2 = Phidget_openWaitForAttachment((PhidgetHandle)motor_right, 5000);
		if (res2 != EPHIDGET_OK)
		{		
			std::cout << "Waiting for right motor..." << std::endl;		
			std::this_thread::sleep_for(1s);
		}
	} while (res2 != EPHIDGET_OK);	


	PhidgetDCMotor_setAcceleration(motor_left, 20);
	PhidgetDCMotor_setAcceleration(motor_right, 20);


	std::cout << "Ready!" << std::endl;	
	

	struct js_event event;
	while (read_event(js, &event) == 0)
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

	return 0;
}