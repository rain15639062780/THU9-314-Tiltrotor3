/****************************************************************************
 *
 *   Copyright (c) 2015 Andrew Tridgell. All rights reserved.
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
 * @file pwm_input.cpp
 *
 * PWM input driver based on earlier driver from Evan Slatyer,
 * which in turn was based on drv_hrt.c
 *
 * @author: Andrew Tridgell
 * @author: Ban Siesta <bansiesta@gmail.com>
 */

/**********************************************
*2018-12-21 rain
*@file pwm_capture.cpp  cap_1
*修改px4源码pwm_input.cpp,使其适配cap口
*FMU_CAP1-------TIM2_CH1_IN-------PA5
*FMU_CAP2-------TIM2_CH2_IN-------PB3
*FMU_CAP3-------TIM2_CH4_IN-------PB11
*FMU_CAP4在官方给定的引脚对应表中并未查询到（FMUV5）
**********************************************/

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cfloat>

#include <board_config.h>
//#include <drivers/drv_pwm_capture_rain.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/drv_device.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32_gpio.h"
#include "stm32_tim.h"

#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>

#include <lib/ecl/geo/geo.h>
#include <lib/ecl/geo_lookup/geo_mag_declination.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/pwm_capture.h>

#define PWMCAP0_DEVICE_PATH		"/dev/pwmcap0"

extern "C" __EXPORT int pwm_capture_main(int argc, char *argv[]);

class PWMCAP;

namespace pwmcap
{

	PWMCAP *g_dev;

}

class PWMCAP 
{
public:
	PWMCAP();
	virtual ~PWMCAP();
	int start();
	int stop();
	virtual int init();
	int task_main_trampoline(int argc, char *argv[]);
	void task_main();


private:

	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */

};



PWMCAP::PWMCAP()
{
	
}

PWMCAP::~PWMCAP()
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

	pwmcap::g_dev = nullptr;

}



int
PWMCAP::start(){
	
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("pwmcap",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ESTIMATOR-1,
					   2000,
					   (px4_main_t)&PWMCAP::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warnx("task start failed");
		return -errno;
	}

	return OK;	
	
}

int
PWMCAP::stop()
{
	
	_task_should_exit = true;
	return OK;	
	
}

int PWMCAP::init(){
	
	return OK;
}



int PWMCAP::task_main_trampoline(int argc, char *argv[])
{
	pwmcap::g_dev->task_main();
	return 0;
}


void PWMCAP::task_main()
{
	int i = 0;
	while (!_task_should_exit) {

		printf("PWMCAP::task_main = %d\n",i++);
		usleep(1000);

	}
}





/*
 * driver entry point
 */
int pwm_capture_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("unrecognized command, try 'start', 'stop'");
		return -1;
	}

	//const char *verb = argv[1];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		
		if (pwmcap::g_dev != nullptr) {
		warnx("driver already started");
		}

		pwmcap::g_dev = new PWMCAP();

		if (pwmcap::g_dev == nullptr) {
			warnx("driver allocation failed");
		}


		if (OK != pwmcap::g_dev->start()){
			delete pwmcap::g_dev;
			pwmcap::g_dev = nullptr;
			PX4_ERR("start failed");
			return 1;
		}
		
		return 0;	
	}

	if (!strcmp(argv[1], "stop")) {
		if (pwmcap::g_dev == nullptr) {
			warnx("not running");
			return 1;
		}

		
		delete pwmcap::g_dev;
		pwmcap::g_dev = nullptr;
		return 0;
	}


	warnx("unrecognized command ");
	warnx("unrecognized command, try 'start', 'stop'");
	return 1;
}

