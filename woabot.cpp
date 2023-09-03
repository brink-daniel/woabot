#include <atomic>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <iostream>
#include <thread>

#include <phidget22.h>

#include "xbox360.h"

using namespace std::chrono_literals;

int left_power = 100;
int right_power = 100;
int forward_power = 0;
int reverse_power = 0;
int power_calibrate = 0;

std::atomic<bool> request_terminate(false);
std::atomic<bool> request_debug(false);
std::atomic<double> request_velocity_left (0);
std::atomic<double> request_velocity_right (0);

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

void PowerBalanceCalibrate(signed short value)
{
	if (abs(value) != 100)
	{
		return;
	}

	if (value > 0)
	{
		// steer more right
		power_calibrate += 1;

		if (power_calibrate > 50)
		{
			power_calibrate = 50;
		}
	}
	else
	{
		// steer more left
		power_calibrate -= 1;

		if (power_calibrate < -50)
		{
			power_calibrate = -50;
		}
	}

	if (request_debug)
	{
		std::cout << "Calibrate: " << power_calibrate << std::endl;
	}
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

double StepVelocity(double value)
{
	value = value * 10;
	value = std::round(value);
	value = value / 10;
	return value;
}

void RequestVelocityUpdate()
{	
	double d_throttle = forward_power - reverse_power;
	double d_max_power = 100;
	double d_left_power = left_power;
	double d_right_power = right_power;
	double d_left_power_reduce_by = 0;
	double d_right_power_reduce_by = 0;


	if (power_calibrate > 0)
	{
		// steer more right
		d_left_power_reduce_by = 0;
		d_right_power_reduce_by = power_calibrate;
	}
	else if (power_calibrate < 0)
	{
		// steer more left
		d_left_power_reduce_by = abs(power_calibrate);
		d_right_power_reduce_by = 0;
	}
	else
	{
		d_left_power_reduce_by = 0;
		d_right_power_reduce_by = 0;
	}


	double left_velocity = (d_throttle / d_max_power) * (d_left_power / d_max_power);
	left_velocity = MinMaxVelocity(left_velocity);
	left_velocity = StepVelocity(left_velocity);
	if (left_velocity == -0)
	{
		left_velocity = 0;
	}

	if (left_velocity > 0)
	{
		left_velocity -= (d_left_power_reduce_by / d_max_power);
	}
	else if (left_velocity < 0)
	{
		left_velocity += (d_left_power_reduce_by / d_max_power);
	}


	
	double right_velocity = (d_throttle / d_max_power) * (d_right_power / d_max_power);
	right_velocity = MinMaxVelocity(right_velocity);
	right_velocity = StepVelocity(right_velocity);
	if (right_velocity == -0)
	{
		right_velocity = 0;
	}

	if (right_velocity > 0)
	{
		right_velocity -= (d_right_power_reduce_by / d_max_power);
	}
	else if (right_velocity < 0)
	{
		right_velocity += (d_right_power_reduce_by / d_max_power);
	}



	if (request_velocity_left != left_velocity)
	{
		request_velocity_left = left_velocity;
	}

	if (request_velocity_right != right_velocity)
	{
		request_velocity_right = right_velocity;
	}
}

void ProcessButtonEvent(unsigned char event_number, signed short event_value)
{
	auto a = static_cast<button>(event_number);
	switch (a)
	{
	case A:
		break;
	case B:
		if (event_value == 1)
		{
			power_calibrate = 0;
		}
		break;
	case X:
		if (event_value == 1)
		{
			std::cout << "Terminate requested" << std::endl;
			request_terminate = true;
		}
		break;
	case Y:
		if (event_value == 1)
		{
			request_debug = !request_debug;
			if (request_debug)
			{
				std::cout << "Debug output enabled" << std::endl;
			}
			else
			{
				std::cout << "Debug output disabled" << std::endl;
			}
		}
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
	RequestVelocityUpdate();
}

void ProcessAxisEvent(unsigned char event_number, signed short event_value)
{
	axis a = static_cast<axis>(event_number);
	int v = axis_value(a, event_value);
	int p = axis_deadzone_filter(a, v);

	switch (a)
	{
	case LeftStick_X:
		PowerBalance(p);
		break;
	case LeftStick_Y:
		break;
	case RightTrigger:
		forward_power = p;
		break;
	case LeftTrigger:
		reverse_power = p;
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
	RequestVelocityUpdate();
}



int main(int argc, char *argv[])
{	
	std::cout << "Woabot" << std::endl;
	
	int js = -1;	
	do
	{
		std::cout << "Waiting for controller..." << std::endl;
		js = open("/dev/input/js0", O_RDONLY);
		if (js == -1)
		{			
			std::this_thread::sleep_for(1s);
		}
	} while (js == -1);


	std::thread motor_control_thread = std::thread([] {

		double current_velocity_left = 0;
		double current_velocity_right = 0;

		PhidgetDCMotorHandle motor_left;
		PhidgetDCMotorHandle motor_right;

		PhidgetDCMotor_create(&motor_left);
		PhidgetDCMotor_create(&motor_right);

		Phidget_setChannel((PhidgetHandle)motor_left, 0);
		Phidget_setChannel((PhidgetHandle)motor_right, 1);
				
		PhidgetReturnCode res1;
		do
		{
			std::cout << "Waiting for Left Motor..." << std::endl;
			res1 = Phidget_openWaitForAttachment((PhidgetHandle)motor_left, 5000);
			if (res1 != EPHIDGET_OK)
			{
				std::this_thread::sleep_for(1s);
			}
		} while (res1 != EPHIDGET_OK && !request_terminate);

		PhidgetReturnCode res2;
		do
		{
			std::cout << "Waiting for Right Motor..." << std::endl;
			res2 = Phidget_openWaitForAttachment((PhidgetHandle)motor_right, 5000);
			if (res2 != EPHIDGET_OK)
			{				
				std::this_thread::sleep_for(1s);
			}
		} while (res2 != EPHIDGET_OK && !request_terminate);

		PhidgetDCMotor_setTargetVelocity(motor_left, 0);
		PhidgetDCMotor_setTargetVelocity(motor_right, 0);

		double maxAcceleration_left;
		PhidgetDCMotor_getMaxAcceleration(motor_left, &maxAcceleration_left);
		maxAcceleration_left = maxAcceleration_left * 0.5;
		PhidgetDCMotor_setAcceleration(motor_left, maxAcceleration_left);

		double maxAcceleration_right;
		PhidgetDCMotor_getMaxAcceleration(motor_right, &maxAcceleration_right);
		maxAcceleration_right = maxAcceleration_right * 0.5;
		PhidgetDCMotor_setAcceleration(motor_right, maxAcceleration_right);

		std::cout << "Ready!" << std::endl;

		while (!request_terminate)
		{
			double r_velocity_left = request_velocity_left;
			double r_velocity_right = request_velocity_right;
			
			if ((current_velocity_left != r_velocity_left)
				|| (current_velocity_right != r_velocity_right))
			{
				PhidgetReturnCode res_left;
				PhidgetReturnCode res_right;

				do
				{
					if (current_velocity_left != r_velocity_left)
					{
						res_left = PhidgetDCMotor_setTargetVelocity(motor_left, r_velocity_left);
						if (res_left == EPHIDGET_OK)
						{
							current_velocity_left = r_velocity_left;
						}
					}

					if (current_velocity_right != r_velocity_right)
					{
						res_right = PhidgetDCMotor_setTargetVelocity(motor_right, r_velocity_right);
						if (res_right == EPHIDGET_OK)
						{
							current_velocity_right = r_velocity_right;
						}
					}

				} while ((res_left != EPHIDGET_OK || res_right != EPHIDGET_OK) && !request_terminate);

				if (request_debug)
				{
					std::cout << "Left: " << current_velocity_left << " Right: " << current_velocity_right << std::endl;
				}
			}

			std::this_thread::sleep_for(10ms);
		}

		PhidgetDCMotor_setTargetVelocity(motor_left, 0);
		PhidgetDCMotor_setTargetVelocity(motor_right, 0);

		PhidgetDCMotor_delete(&motor_left);
		PhidgetDCMotor_delete(&motor_right);

	});		

	struct js_event event;
	while (read_event(js, &event) == 0 && !request_terminate)
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

	motor_control_thread.join();
	std::cout << "Exit" << std::endl;
	return 0;
}