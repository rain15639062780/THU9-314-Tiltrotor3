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
#include <px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <board_config.h>
//#include <drivers/drv_pwm_capture_rain.h>
#include <drivers/pwm_capture/pwm_capture_config.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32_gpio.h"
#include "stm32_tim.h"
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/pwm_capture.h>

#include <drivers/drv_device.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define PWMCAP0_DEVICE_PATH		"/dev/pwmcap0"

extern "C" __EXPORT int pwm_capture_main(int argc, char *argv[]);



class PWMCAP : device::CDev
{
public:
	PWMCAP();
	virtual ~PWMCAP();

	virtual int init();
	virtual int open(struct file *filp);
	virtual ssize_t read(struct file *filp, char *buffer, size_t buflen);


	void publish(uint16_t status, uint32_t period, uint32_t pulse_width);
	int task_main_create();
	int task_main_trampoline(int argc, char *argv[]);

	void task_main();


private:
	uint32_t _error_count;
	hrt_abstime _last_poll_time;
	hrt_abstime _last_read_time;
	ringbuffer::RingBuffer *_reports;
	bool _timer_started;

	bool		_task_should_exit ;		/**< if true, task should exit */
	int		_control_task ;			/**< task handle for task */
	//int     fd_pwmcap;


	void _timer_init(void);



};

static int pwmcap_tim_isr(int irq, void *context, void *arg);
static void pwmcap_start();
static void pwmcap_test(void);
static void pwmcap_usage(void);


static PWMCAP *g_dev;
static int fd_pwmcap = -1;

PWMCAP::PWMCAP() :
	CDev("pwmcap", PWMCAP0_DEVICE_PATH),
	_error_count(0),
	_reports(nullptr),
	_timer_started(false),
	_task_should_exit(false),
	_control_task(-1)
	//fd_pwmcap(-1)
{
}

PWMCAP::~PWMCAP()
{
	if (_reports != nullptr) {
		delete _reports;
	}

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

	g_dev = nullptr;

	
}

/*
 * initialise the driver. This doesn't actually start the timer (that
 * is done on open). We don't start the timer to allow for this driver
 * to be started in init scripts when the user may be using the input
 * pin as PWM output
 */
int
PWMCAP::init()
{
	/* we just register the device in /dev, and only actually
	 * activate the timer when requested to when the device is opened */
	CDev::init();

	_reports = new ringbuffer::RingBuffer(2, sizeof(struct pwm_capture_s));

	if (_reports == nullptr) {
		return -ENOMEM;
	}


	return OK;
}

/*
 * Initialise the timer we are going to use.
 */
void PWMCAP::_timer_init(void)
{
	/* run with interrupts disabled in case the timer is already
	 * setup. We don't want it firing while we are doing the setup */
	irqstate_t flags = px4_enter_critical_section();

	/* configure input pin */
	px4_arch_configgpio(GPIO_PWM_IN);

	// XXX refactor this out of this driver
	/* configure reset pin */
	px4_arch_configgpio(GPIO_VDD_RANGEFINDER_EN);

	/* claim our interrupt vector */
	irq_attach(PWMCAP_TIMER_VECTOR, pwmcap_tim_isr, NULL);

	/* Clear no bits, set timer enable bit.*/
	modifyreg32(PWMCAP_TIMER_POWER_REG, 0, PWMCAP_TIMER_POWER_BIT);

	/* disable and configure the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = DIER_PWMCAP_A;
	rCCER = 0;		/* unlock CCMR* registers */
	rCCMR1 = CCMR1_PWMCAP;
	rCCMR2 = CCMR2_PWMCAP;
	rSMCR = SMCR_PWMCAP_1;	/* Set up mode */
	rSMCR = SMCR_PWMCAP_2;	/* Enable slave mode controller */
	rCCER = CCER_PWMCAP;
	rDCR = 0;

	/* for simplicity scale by the clock in MHz. This gives us
	 * readings in microseconds which is typically what is needed
	 * for a PWM input driver */
	uint32_t prescaler = PWMCAP_TIMER_CLOCK / 1000000UL;

	/*
	 * define the clock speed. We want the highest possible clock
	 * speed that avoids overflows.
	 */
	rPSC = prescaler - 1;

	/* run the full span of the counter. All timers can handle
	 * uint16 */
	rARR = UINT16_MAX;

	/* generate an update event; reloads the counter, all registers */
	rEGR = GTIM_EGR_UG;

	/* enable the timer */
	rCR1 = GTIM_CR1_CEN;

	/* enable interrupts */
	up_enable_irq(PWMCAP_TIMER_VECTOR);

	px4_leave_critical_section(flags);

	_timer_started = true;
}


/*
 * hook for open of the driver. We start the timer at this point, then
 * leave it running
 */
int
PWMCAP::open(struct file *filp)
{
	if (g_dev == nullptr) {
		return -EIO;
	}

	int ret = CDev::open(filp);

	if (ret == OK && !_timer_started) {
		g_dev->_timer_init();
	}

	return ret;
}



/*
 * read some samples from the device
 */
ssize_t
PWMCAP::read(struct file *filp, char *buffer, size_t buflen)
{
	_last_read_time = hrt_absolute_time();

	unsigned count = buflen / sizeof(struct pwm_capture_s);
	struct pwm_capture_s *buf = reinterpret_cast<struct pwm_capture_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	while (count--) {
		if (_reports->get(buf)) {
			ret += sizeof(struct pwm_capture_s);
			buf++;
		}
	}

	/* if there was no data, warn the caller */
	return ret ? ret : -EAGAIN;
}

/*
 * publish some data from the ISR in the ring buffer
 */
void PWMCAP::publish(uint16_t status, uint32_t period, uint32_t pulse_width)
{
	/* if we missed an edge, we have to give up */
	if (status & SR_OVF_PWMCAP) {
		_error_count++;
		return;
	}

	_last_poll_time = hrt_absolute_time();

	struct pwm_capture_s pwmcap_report;
	pwmcap_report.timestamp = _last_poll_time;
	pwmcap_report.error_count = _error_count;
	pwmcap_report.period = period;
	pwmcap_report.pulse_width = pulse_width;

	_reports->force(&pwmcap_report);
}




/*
 * Handle the interrupt, gathering pulse data
 */
static int pwmcap_tim_isr(int irq, void *context, void *arg)
{
	uint16_t status = rSR;
	uint32_t period = rCCR_PWMCAP_A;
	uint32_t pulse_width = rCCR_PWMCAP_B;

	/* ack the interrupts we just read */
	rSR = 0;

	if (g_dev != nullptr) {
		g_dev->publish(status, period, pulse_width);
	}

	return OK;
}

/*
 * start the driver
 */
static void pwmcap_start()
{
	if (g_dev != nullptr) {
		errx(1, "driver already started");
	}

	g_dev = new PWMCAP();

	if (g_dev == nullptr) {
		errx(1, "driver allocation failed");
	}

	if (g_dev->init() != OK) {
		errx(1, "driver init failed");
	}


	fd_pwmcap = open(PWMCAP0_DEVICE_PATH, O_RDONLY);

	if (fd_pwmcap == -1) {
		errx(1, "Failed to open device");
	}



	if (g_dev->task_main_create() != OK) {
		errx(1, "task create failed");
	}

		

	exit(0);
}

int PWMCAP::task_main_create()
{
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


int PWMCAP::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
	return 0;
}

void PWMCAP::task_main()
{
	
	//fd_pwmcap = open(PWMCAP0_DEVICE_PATH, O_RDONLY);

//	if (fd_pwmcap == -1) {
//		errx(1, "Failed to open device");
//	}
	
	//int i = 0;
	while (!_task_should_exit) {
		struct pwm_capture_s buf;

		if (::read(fd_pwmcap, &buf, sizeof(buf)) == sizeof(buf)) {
				printf("period=%u width=%u error_count=%u\n",
					   (unsigned)buf.period,
					   (unsigned)buf.pulse_width,
					   (unsigned)buf.error_count);

			} else {
				// no data, retry in 2 ms 
				usleep(2000);
			}

			//printf("PWMCAP::task_main = %d\n",i++);
			//usleep(1000);
		}
		
		
	close(fd_pwmcap);
	exit(0);

}


/*
 * test the driver
 */
static void pwmcap_test(void)
{
	int fd = open(PWMCAP0_DEVICE_PATH, O_RDONLY);

	if (fd == -1) {
		errx(1, "Failed to open device");
	}

	uint64_t start_time = hrt_absolute_time();

	printf("Showing samples for 5 seconds\n");

	while (hrt_absolute_time() < start_time + 5U * 1000UL * 1000UL) {
		struct pwm_capture_s buf;

		if (::read(fd, &buf, sizeof(buf)) == sizeof(buf)) {
			printf("period=%u width=%u error_count=%u\n",
			       (unsigned)buf.period,
			       (unsigned)buf.pulse_width,
			       (unsigned)buf.error_count);

		} else {
			/* no data, retry in 2 ms */
			::usleep(2000);
		}
	}

	close(fd);
	exit(0);
}




static void pwmcap_usage()
{
	PX4_ERR("unrecognized command, try 'start', 'info', 'reset' or 'test'");
}

/*
 * driver entry point
 */
int pwm_capture_main(int argc, char *argv[])
{
	if (argc < 2) {
		pwmcap_usage();
		return -1;
	}

	const char *verb = argv[1];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		pwmcap_start();
	}


	/*
	 * print test results
	 */
	if (!strcmp(verb, "test")) {
		pwmcap_test();
	}


	pwmcap_usage();
	return -1;
}

