/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include "telecom_vitesse.hpp"
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/safety.h>

int TelecomVitesse::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int TelecomVitesse::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int TelecomVitesse::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

TelecomVitesse *TelecomVitesse::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	//int myoptind = 1;
	//int ch;
	//const char *myoptarg = nullptr;

	// parse CLI arguments
	/*while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}*/

	if (error_flag) {
		return nullptr;
	}

	TelecomVitesse *instance = new TelecomVitesse(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

TelecomVitesse::TelecomVitesse(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}




// function that handles Steer and Speed control
void TelecomVitesse::roverSteerSpeed(roverControl control, vehicle_attitude_setpoint_s &_att_sp, vehicle_attitude_s &att)
{
	PX4_INFO("set speed");
	// Converting steering value from percent to euler angle
	control.steer *= -60.0f; //max turn angle 60 degree
	control.steer *= (float)3.14159/180; // change to radians

	// Get current attitude quaternion
	matrix::Quatf current_qe{att.q[0], att.q[1], att.q[2], att.q[3]};

	// Converting the euler angle into a quaternion for vehicle_attitude_setpoint
	matrix::Eulerf euler{0.0, 0.0, control.steer};
	matrix::Quatf qe{euler};

	// Create new quaternion from the difference of current vs steering
	matrix::Quatf new_qe;
	new_qe = current_qe * qe.inversed();

	// Throttle control of the rover
	_att_sp.thrust_body[0] = control.speed;

	// Steering control of the Rover
	new_qe.copyTo(_att_sp.q_d);

}

void TelecomVitesse::run()
{
	// Example: run the loop synchronized to the sensor_combined topic publication
	//int sensor_combined_sub = orb_subscribe(ORB_ID(pixy_vector));

	//px4_pollfd_struct_t fds[1];
	//fds[0].fd = sensor_combined_sub;
	//fds[0].events = POLLIN;

	// initialize parameters
	roverControl control{};
	control.steer *= -60.0f; //max turn angle 60 degree
	control.steer *= (float)3.14159/180; // change to radians
	control.speed = SPEED_FAST;

	struct vehicle_control_mode_s				_control_mode;
	memset(&_control_mode, 0, sizeof(_control_mode));
				// Setting vehicle to attitude control mode
			_control_mode.flag_control_manual_enabled 	= false;
			_control_mode.flag_control_attitude_enabled 	= true;
			_control_mode.flag_control_velocity_enabled 	= false;
			_control_mode.flag_control_position_enabled	= false;
	parameters_update(true);
	while (!should_exit()) {

        printf("runing...\n");
        px4_usleep(1_s);
		
        //fds[0].events = POLLIN;

		// wait for up to 1000ms for data
	/**	int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			struct sensor_combined_s sensor_combined;
			orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
			// TODO: do something with the data...

		}

		parameters_update();*/

	}

	//orb_unsubscribe(ORB_ID(pixy_vector));
}

void TelecomVitesse::parameters_update(bool force)
{
	// check for parameter updates
	/*if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}*/
}

int TelecomVitesse::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "telecom_vitesse");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int telecom_vitesse_main(int argc, char *argv[])
{
	return TelecomVitesse::main(argc, argv);
}