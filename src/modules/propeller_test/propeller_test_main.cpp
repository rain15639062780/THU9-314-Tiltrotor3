

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
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/adc_report.h>



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

	bool actuator_controls_poll();
	bool actuator_outputs_poll();
	bool pwm_capture_poll();
	bool adc_capture_poll();

	void propeller_data_deal();

	int 	_actuator_control_0_sub = -1;

	
//	int 	_actuator_outputs_0_sub = -1;
	int 	_actuator_outputs_sub[2]{ };	


	
	int 	_pwm_capture_sub = -1;
	int 	_adc_capture_sub = -1;

	struct actuator_controls_s		_actuator_controls {};		/**< actuator controls */
	struct actuator_outputs_s		_actuator_outputs[2] {};	  /*actustor pwmouts*/
	struct pwm_capture_s			_pwm_capture {};	/**< battery status */
	struct adc_report_s			_adc_capture {};	//< gyro data before thermal correctons and



	struct propeller_data_s{
		
		float act_control[4];	//control: roll,pitch,yaw,thrust
		float act_output[4];	//channel outputs:s0,s1,s2,s3
		uint32_t pwm_period;	//unit: us
		float voltage;//adc_report.channel_value[10];
		float current;//adc_report.channel_value[4];
	};

	struct propeller_data_uart_s{
		
		uint16_t act_control[4];	//control: roll,pitch,yaw,thrust
		uint16_t act_output[4];	//channel outputs:s0,s1,s2,s3
		uint32_t pwm_period;	//unit: us
		uint16_t voltage;//adc_report.channel_value[10];
		uint16_t current;//adc_report.channel_value[4];
	};


	struct propeller_data_s propeller_data_raw;//保存原始数据
	struct propeller_data_uart_s propeller_data_send;//将原始数据转换成无符号整型类型数据

	uint8_t send_buf[sizeof(propeller_data_uart_s)];//定义发送buffer
	


};



PROPELLER::PROPELLER()
{

	for (unsigned i = 0; i < (sizeof(_actuator_outputs_sub) / sizeof(_actuator_outputs_sub[0])); i++) {
	
		_actuator_outputs_sub[i] = -1;
	}


	
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


bool
PROPELLER::actuator_controls_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_actuator_control_0_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls), _actuator_control_0_sub, &_actuator_controls);
	}

	return updated;
}

bool
PROPELLER::actuator_outputs_poll()
{
	/* check if there is a new message */
	bool updated;
	for (unsigned i = 0; i < (sizeof(_actuator_outputs_sub) / sizeof(_actuator_outputs_sub[0])); i++) {
	
		orb_check(_actuator_outputs_sub[i], &updated);
	
		if (updated) {
			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub[i], &_actuator_outputs[i]);
		}
	}

	return updated;
}



bool
PROPELLER::pwm_capture_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_pwm_capture_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(pwm_capture), _pwm_capture_sub, &_pwm_capture);
	}

	return updated;
}


bool
PROPELLER::adc_capture_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_adc_capture_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(adc_report), _adc_capture_sub, &_adc_capture);
	}

	return updated;
}



void
PROPELLER::propeller_data_deal(){

	//获取原始数据
	propeller_data_raw.act_control[0] = _actuator_controls.control[0];
	propeller_data_raw.act_control[1] = _actuator_controls.control[1];
	propeller_data_raw.act_control[2] = _actuator_controls.control[2];
	propeller_data_raw.act_control[3] = _actuator_controls.control[3];
	
	
	propeller_data_raw.act_output[0] = _actuator_outputs[0].output[0];
	propeller_data_raw.act_output[1] = _actuator_outputs[0].output[1];
	propeller_data_raw.act_output[2] = _actuator_outputs[0].output[2];
	propeller_data_raw.act_output[3] = _actuator_outputs[0].output[3];
	
	propeller_data_raw.pwm_period = _pwm_capture.period;
	propeller_data_raw.voltage = _adc_capture.channel_value[10];
	propeller_data_raw.current = _adc_capture.channel_value[4];

	printf("propeller_data_raw\n");
	printf("control[3] = %f act_output[0] = %f period = %d volt = %f current = %f \n\n\n",                              
		(double)propeller_data_raw.act_control[3],
		(double)propeller_data_raw.act_output[0],
		propeller_data_raw.pwm_period,
		(double)propeller_data_raw.voltage,
		(double)propeller_data_raw.current);



	
	//数据类型转换
	propeller_data_send.act_control[0] = (uint16_t)(((double)propeller_data_raw.act_control[0]+0.0005)*1000);
	propeller_data_send.act_control[1] = (uint16_t)(((double)propeller_data_raw.act_control[1]+0.0005)*1000);
	propeller_data_send.act_control[2] = (uint16_t)(((double)propeller_data_raw.act_control[2]+0.0005)*1000);
	propeller_data_send.act_control[3] = (uint16_t)(((double)propeller_data_raw.act_control[3]+0.0005)*1000);
	
	propeller_data_send.act_output[0] =  (uint16_t)(_actuator_outputs[0].output[0]+(float)0.5);
	propeller_data_send.act_output[1] =  (uint16_t)(_actuator_outputs[0].output[1]+(float)0.5);
	propeller_data_send.act_output[2] =  (uint16_t)(_actuator_outputs[0].output[2]+(float)0.5);
	propeller_data_send.act_output[3] =  (uint16_t)(_actuator_outputs[0].output[3]+(float)0.5);
	
	propeller_data_send.pwm_period = _pwm_capture.period+1;
	propeller_data_send.voltage = (uint16_t)((double)_adc_capture.channel_value[10]/3.3*4096.0+0.5);//转换为adc采样输出值
	propeller_data_send.current = (uint16_t)((double)_adc_capture.channel_value[4]/6.6*4096.0*2.0+0.5);//转换为adc采样输出值

	printf("propeller_data_send\n");
	printf("control[3] = %u act_output[0] = %u period = %u volt = %u current = %u\n\n\n",                              
		propeller_data_send.act_control[3],
		propeller_data_send.act_output[0],
		propeller_data_send.pwm_period,
		propeller_data_send.voltage,
		propeller_data_send.current);



	uint8_t i = 0;
	
	//propeller_data_send.act_control
	for(i = 0; i < 8; i += 2){
		send_buf[i] = (uint8_t)((propeller_data_send.act_control[i/2] & 0xff00) >> 8);//高字节
		send_buf[i+1] = (uint8_t)(propeller_data_send.act_control[i/2] & 0x00ff);//低字节
	}
	
	//propeller_data_send.act_output
	for(i = 0; i < 8; i += 2){
		send_buf[i+8] = (uint8_t)((propeller_data_send.act_output[i/2] & 0xff00) >> 8);//高字节
		send_buf[i+1+8] = (uint8_t)(propeller_data_send.act_output[i/2] & 0x00ff);//低字节
	}
	
	//propeller_data_send.pwm_period
	send_buf[16] = (uint8_t)((propeller_data_send.pwm_period & 0xff000000) >> 24);
	send_buf[17] = (uint8_t)((propeller_data_send.pwm_period & 0x00ff0000) >> 16);
	send_buf[18] = (uint8_t)((propeller_data_send.pwm_period & 0x0000ff00) >> 8);
	send_buf[19] = (uint8_t)(propeller_data_send.pwm_period & 0x000000ff);
	
	//propeller_data_send.voltage
	send_buf[20] = (uint8_t)((propeller_data_send.voltage & 0xff00) >> 8);//高字节
	send_buf[21] = (uint8_t)(propeller_data_send.voltage & 0x00ff);//低字节
	
	//propeller_data_send.current
	send_buf[22] = (uint8_t)((propeller_data_send.current & 0xff00) >> 8);//高字节
	send_buf[23] = (uint8_t)(propeller_data_send.current & 0x00ff);//低字节


	
	for(i=0; i<sizeof(propeller_data_uart_s); i++){
		printf("send_buf[%d]=%d\n",i,send_buf[i]);

	}

	printf("\n\n");
	
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

	_actuator_control_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));
//	_actuator_outputs_0_sub = orb_subscribe(ORB_ID(actuator_outputs));

	_pwm_capture_sub = orb_subscribe(ORB_ID(pwm_capture));
	_adc_capture_sub = orb_subscribe(ORB_ID(adc_report));

	for (unsigned i = 0; i < (sizeof(_actuator_outputs_sub) / sizeof(_actuator_outputs_sub[0])); i++) {
		_actuator_outputs_sub[i] = orb_subscribe_multi(ORB_ID(actuator_outputs), i);
	}



	unsigned int i = 0;
	while (!_task_should_exit) {

		bool control_update =  actuator_controls_poll();
		bool outputs_update = actuator_outputs_poll();
		bool pwm_update = pwm_capture_poll();
		bool adc_update = adc_capture_poll();

//		struct actuator_controls_s		_actuator_controls {};		/**< actuator controls */
//		struct actuator_outputs_s			_actuator_outputs {};	/**< vehicle status */
//		struct pwm_capture_s			_pwm_capture {};	/**< battery status */
//		struct adc_report_s 		_adc_capture {};	//< gyro data before thermal correctons and

//		struct propeller_data_s propeller_data_raw;//保存原始数据
//		struct propeller_data_uart_s propeller_data_send;//将原始数据转换成无符号整型类型数据
//		send_buf


		if(control_update&&outputs_update&&pwm_update&&adc_update){


		}

		
		if(control_update){
	//		for( i = 0; i<8; i++){
	//		printf("control[%d]= %f\n",i,(double)_actuator_controls.control[i]*1000);	
	//		}
			printf("control_update\n");
		}
		
		if(outputs_update){
		//	for( i = 0; i<8; i++){
		//	printf("output[%d]= %f\n",i,(double)_actuator_outputs[1].output[i]);

		//	}
		//	printf("padding= %f\n",_actuator_outputs[1]._padding0[0]);
		
			printf("outputs_update\n");
		}
		
		if(pwm_update){
		//	printf("pwm_capture.period = %u\n",(unsigned)_pwm_capture.period);
			printf("pwm_update\n");
		}
		
		if(adc_update){
		//	printf("adc_value[6] = %u\n",(double)_adc_capture.channel_value[6]);
		//		for( i = 0; i<12; i++){
		//		printf("adc_value[%d]= %f\n",i,(double)_adc_capture.channel_value[i]*1000000000);		
		//		}
			printf("adc_update\n");
		}
	
		propeller_data_deal();




		printf("PROPELLER::task_main = %d\n\n\n",i++);
		usleep(500000);

	}


	orb_unsubscribe(_actuator_control_0_sub);
	//orb_unsubscribe(_actuator_outputs_0_sub);
	orb_unsubscribe(_pwm_capture_sub);
	orb_unsubscribe(_adc_capture_sub);

	for (unsigned j = 0; j < (sizeof(_actuator_outputs_sub) / sizeof(_actuator_outputs_sub[0])); j++) {
		
		orb_unsubscribe(_actuator_outputs_sub[j]);
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

