/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

/**
 * @file VTOL_att_control_main.cpp
 * Implementation of an attitude controller for VTOL airframes. This module receives data
 * from both the fixed wing- and the multicopter attitude controllers and processes it.
 * It computes the correct actuator controls depending on which mode the vehicle is in (hover,forward-
 * flight or transition). It also publishes the resulting controls on the actuator controls topics.
 *
 * @author Roman Bapst 		<bapstr@ethz.ch>
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler	<thomasgubler@gmail.com>
 * @author David Vorsin		<davidvorsin@gmail.com>
 * @author Sander Smeets	<sander@droneslab.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
 */
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <parameters/param.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/sever_nozzle_rx.h>
#include <uORB/topics/sever_nozzle_tx.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tecs_status.h>
#include <systemlib/mavlink_log.h>



extern "C" __EXPORT int sever_nozzle_control_main(int argc, char *argv[]);



class sever_nozzle
{
public:

	sever_nozzle();
	~sever_nozzle();

	int start();	/* start the task and return OK on success */
	int stop();



private:
//******************flags & handlers******************************************************
	bool	_task_should_exit{false};
	int	_control_task{-1};		//task handle for VTOL attitude controller

	/* handlers for subscriptions */


	int	_manual_control_sp_sub{-1};	//manual control setpoint subscription
	int _sever_nozzle_rx_sub{-1}; //manual control setpoint subscription


	//handlers for publishers
	orb_advert_t	_sever_nozzle_tx_pub{nullptr};		//input for the mixer (roll,pitch,yaw,thrust)




//*******************data containers***********************************************************

	manual_control_setpoint_s		_manual_control_sp{}; //manual control setpoint
	sever_nozzle_rx_s		_sever_nozzle_rx{}; //
	sever_nozzle_tx_s		_sever_nozzle_tx{}; //


//*****************Member functions***********************************************************************

	void 		task_main();	//main task
	static int	task_main_trampoline(int argc, char *argv[]);	//Shim for calling task_main from task_create.

	void		vehicle_manual_poll();			//Check for changes in manual inputs.
	void		sever_nozzle_rx_poll();			//Check for changes in manual inputs.

};



namespace sever_nozzle_control
{
	sever_nozzle *g_control;
}



/**
 * Constructor
 */
sever_nozzle::sever_nozzle()
{
	fflush(stdout);

	//_task_should_exit = true;
}

/**
 * Destructor
 */
sever_nozzle::~sever_nozzle()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	sever_nozzle_control::g_control = nullptr;

}

/**
 * Check for changes in manual inputs.
 */
void sever_nozzle::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);

	//获取数据

	//printf("_manual_control_sp.flaps = %.4f\n",(double) _manual_control_sp.flaps);
	//printf("_manual_control_sp.aux1 = %.4f\n",(double) _manual_control_sp.aux1);
}
	
}


/**
 * Check for changes in manual inputs.
 */
void sever_nozzle::sever_nozzle_rx_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_sever_nozzle_rx_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(sever_nozzle_rx), _sever_nozzle_rx_sub, &_sever_nozzle_rx);
		printf("_sever_nozzle_rx.pos = %x\n",_sever_nozzle_rx.pos);
	}
	
}




int
sever_nozzle::task_main_trampoline(int argc, char *argv[])
{
	sever_nozzle_control::g_control->task_main();
	return 0;
}


int
sever_nozzle::start()
{
	/* start the task */
	_control_task = px4_task_spawn_cmd("sever_nozzle_control",
			SCHED_DEFAULT,
			SCHED_PRIORITY_ATTITUDE_CONTROL + 1,
			1230,
			(px4_main_t)&sever_nozzle::task_main_trampoline,
			nullptr);

	if (_control_task < 0) {
		PX4_WARN("task start failed");
		return -errno;
	}

	return OK;
}


int sever_nozzle::stop(){
	
	_task_should_exit  = true;
	return 0;
}



void sever_nozzle::task_main()
{
	fflush(stdout);

	/* do subscriptions */

	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_sever_nozzle_rx_sub = orb_subscribe(ORB_ID(sever_nozzle_rx));

	_task_should_exit = false;


	while (!_task_should_exit) {
		/* only update parameters if they changed */


		usleep(2000);
		vehicle_manual_poll();
		sever_nozzle_rx_poll();	

		// Advertise/Publish vtol vehicle status
		_sever_nozzle_tx.timestamp = hrt_absolute_time();

		_sever_nozzle_tx.head1 = 0x55;
		_sever_nozzle_tx.head2 = 0xAA;
		_sever_nozzle_tx.pos = (uint16_t)(_manual_control_sp.aux1 *2000);
		_sever_nozzle_tx.timer = 0x00;
		_sever_nozzle_tx.moto_num = 0x02;
		_sever_nozzle_tx.checksum = ((_sever_nozzle_tx.pos & 0xff00 >> 8) + (_sever_nozzle_tx.pos & 0xff) + (_sever_nozzle_tx.moto_num & 0x03));


		if (_sever_nozzle_tx_pub != nullptr) {
		orb_publish(ORB_ID(sever_nozzle_tx), _sever_nozzle_tx_pub, &_sever_nozzle_tx);
		//printf("orb_publish(ORB_ID(sever_nozzle_tx)\n");

		} else {
		_sever_nozzle_tx_pub = orb_advertise(ORB_ID(sever_nozzle_tx), &_sever_nozzle_tx);
		}
		 

		//printf("vehicle_manual_poll success\n");

	}



	PX4_WARN("exit");
	_control_task = -1;
}




int sever_nozzle_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_WARN("usage: sever_nozzle_control {start|stop|status}");
		return 1;
	}



	if (!strcmp(argv[1], "start")) {

		if (sever_nozzle_control::g_control != nullptr) {
			PX4_WARN("already running");
			return 0;
		}

		sever_nozzle_control::g_control = new sever_nozzle;

		if (sever_nozzle_control::g_control == nullptr) {
			PX4_WARN("alloc failed");
			return 1;
		}

		if (OK != sever_nozzle_control::g_control->start()) {
			delete sever_nozzle_control::g_control;
			sever_nozzle_control::g_control = nullptr;
			PX4_WARN("start failed");
			return 1;
		}

		return 0;
	}



	if (!strcmp(argv[1], "stop")) {
		if (sever_nozzle_control::g_control == nullptr) {
			PX4_WARN("not running");
			return 0;
		}

		//_task_should_exit  = true;


		//sever_nozzle_control::g_control->stop();

		delete sever_nozzle_control::g_control;
		sever_nozzle_control::g_control = nullptr;
		return 0;
	}



	if (!strcmp(argv[1], "status")) {

		return 0;
	}

	PX4_WARN("unrecognized command");
	return 1;
}



