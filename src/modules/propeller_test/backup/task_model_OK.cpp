

/**
 * @file propeller_test_main.cpp
 *
 *
 * @author: rain
 */

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



extern "C" __EXPORT int propeller_test_main(int argc, char *argv[]);

class PROPELLER;

namespace propeller
{

	PROPELLER *instance;

}

class PROPELLER 
{
public:
	PROPELLER();
	virtual ~PROPELLER();
	int start();
	int stop();
	virtual int init();
	static int task_main_trampoline(int argc, char *argv[]);
	void task_main();


private:

	bool		_task_should_exit = false;		/**< if true, task should exit */
	int		_control_task = -1;			/**< task handle for task */

};



PROPELLER::PROPELLER()
{
	
}

PROPELLER::~PROPELLER()
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

	propeller::instance = nullptr;

}



int
PROPELLER::start(){
	
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("propeller",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_ESTIMATOR-2,
					   2000,
					   (px4_main_t)&PROPELLER::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warnx("task start failed");
		return -errno;
	}

	return OK;	
	
}

int
PROPELLER::stop()
{
	
	_task_should_exit = true;
	return OK;	
	
}

int PROPELLER::init(){
	
	return OK;
}



int PROPELLER::task_main_trampoline(int argc, char *argv[])
{
	propeller::instance->task_main();
	return 0;
}


void PROPELLER::task_main()
{
	int i = 0;
	while (!_task_should_exit) {

		printf("PROPELLER::task_main = %d\n",i++);
		usleep(200000);

	}
}


/*
 * driver entry point
 */
int propeller_test_main(int argc, char *argv[])
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
		
		if (propeller::instance != nullptr) {
		warnx("driver already started");
		}

		propeller::instance = new PROPELLER();

		if (propeller::instance == nullptr) {
			warnx("driver allocation failed");
		}


		if (OK != propeller::instance->start()){
			delete propeller::instance;
			propeller::instance = nullptr;
			PX4_ERR("start failed");
			return 1;
		}
		
		return 0;	
	}

	if (!strcmp(argv[1], "stop")) {
		if (propeller::instance == nullptr) {
			warnx("not running");
			return 1;
		}

		
		delete propeller::instance;
		propeller::instance = nullptr;
		return 0;
	}


	warnx("unrecognized command ");
	warnx("unrecognized command, try 'start', 'stop'");
	return 1;
}

