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


#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))
extern "C" __EXPORT int sever_nozzle_control_main(int argc, char *argv[]);


#define SIN_PERIOD	10*1000000   //10s
#define STEP_PERIOD	3*1000000   //2s
#define RC2_INPUT_HMAX 50       //转换后最大的角度


class sever_nozzle
{
public:

	sever_nozzle();
	~sever_nozzle();

	int start();	/* start the task and return OK on success */
	int stop();



private:


//喷管角度计算
	double a_nozzle = 25/57.3;
	double a1_nozzle; //喷管第一级转动角度值
	double a2_nozzle; //喷管第二级转动角度值
	double Ky_nozzle; //喷管左右偏转角度，如果保持喷管从垂直到水平变化，不左右偏这个值为零，若左右偏Ky度，实际喷管就偏Ky度
	double Kn_nozzle; //喷管随动角度
	double c1_nozzle; //一级舵机输入角度
	double c2_nozzle[2]; //二级舵机输入角度

	//实际控制舵机的数目
	uint8_t _moto_count;

	//uint64_t time_t1;

	float rc2_input;  //角度阶梯变化角度
	float rc2_array[11];
	uint8_t switch_cmd;

	double run_dt;

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
	void		sever_nozzle_calc();
	float		sever_nozzle_step();
	float 		sever_nozzle_sine();

};



namespace sever_nozzle_control
{
	sever_nozzle *g_control;
}



/**
 * Constructor
 */
sever_nozzle::sever_nozzle():
a_nozzle(25/57.3),
a1_nozzle(0.0),
Ky_nozzle(0.0),
Kn_nozzle(0.0),
c1_nozzle(0.0),
//c2_nozzle(0.0,0.0),
_moto_count(2),
//time_t1(0.0),
rc2_input(0.0),
switch_cmd(0x11),
run_dt(0.0)
{
	fflush(stdout);
	//舵机运动角度对应串口值
	rc2_array[0] = 0.0;
	rc2_array[1] = 10.0;
	rc2_array[2] = 20.0;
	rc2_array[3] = 30.0;
	rc2_array[4] = 40.0;
	rc2_array[5] = 50.0;
	rc2_array[6] = 60.0;
	rc2_array[7] = 70.0;
	rc2_array[8] = 80.0;
	rc2_array[9] = 90.0;
	rc2_array[10] = 100.0;

	//_task_should_exit = true;
	c2_nozzle[0] = 0.0;
	c2_nozzle[1] = 0.0;
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
			usleep(2000);

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
	//printf("return_switch = 0x%2x\n", _manual_control_sp.return_switch);
	//printf("offboard_switch = 0x%2x\n", _manual_control_sp.offboard_switch);

	if(_manual_control_sp.return_switch==3 && _manual_control_sp.offboard_switch==3){switch_cmd = 0x11;}
	else if(_manual_control_sp.return_switch==1 && _manual_control_sp.offboard_switch==3){switch_cmd = 0x01;}
	else if(_manual_control_sp.return_switch==1 && _manual_control_sp.offboard_switch==1){switch_cmd = 0x00;}
	else if(_manual_control_sp.return_switch==3 && _manual_control_sp.offboard_switch==1){switch_cmd = 0x10;}

	//printf("switch_cmd = 0x%2x\n", switch_cmd);
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
		//printf("_sever_nozzle_rx.pos = 0x%4x\n",_sever_nozzle_rx.pos);
	}
	
}

void  sever_nozzle::sever_nozzle_calc()
{
	

//	double a_nozzle = 25/57.3;
//	double a1_nozzle; //喷管第一级转动角度值
//	double a2_nozzle; //喷管第二级转动角度值
//	double Ky_nozzle; //喷管左右偏转角度，如果保持喷管从垂直到水平变化，不左右偏这个值为零，若左右偏Ky度，实际喷管就偏Ky度
//	double Kn_nozzle; //喷管随动角度
//	double  c1_nozzle; //一级舵机输入角度
//	double  c2_nozzle; //二级舵机输入角度
//%%%%%%%%%%%%%%%%%%%


//	Ky_nozzle = (10.0 * (double)_manual_control_sp.aux1)/57.3;//遥控器aux1 -1.0~1.0映射-10~10度 在换成弧度
//	Kn_nozzle = (50.0 * ((double)_manual_control_sp.aux2 + 1.0) )/57.3;//遥控器6 967~1815映射0~100度 在换成弧度


	if(switch_cmd == 0x11)
	{
		Ky_nozzle = (10.0 * (double)_manual_control_sp.aux1)/57.3;//遥控器aux1 -1.0~1.0映射-10~10度 在换成弧度
		Kn_nozzle = (50.0 * ((double)_manual_control_sp.aux2 + 1.0) )/57.3;//遥控器6 967~1815映射0~100度 在换成弧度
	}else if(switch_cmd == 0x01 || switch_cmd == 0x00 )
	{
		//sever_nozzle_step();
		Ky_nozzle = (10.0 * (double)_manual_control_sp.aux1)/57.3;//遥控器aux1 -1.0~1.0映射-10~10度 在换成弧度
		Kn_nozzle = ((double)sever_nozzle_step())/57.3;//遥控器6 967~1815映射0~100度 在换成弧度

		

	}else if(switch_cmd == 0x10)
	{
		//sever_nozzle_sine();

		Ky_nozzle = (10.0 * (double)_manual_control_sp.aux1)/57.3;//遥控器aux1 -1.0~1.0映射-10~10度 在换成弧度
		//Kn_nozzle = (50.0 * (double)sever_nozzle_sine())/57.3;//遥控器6 967~1815映射0~100度 在换成弧度
		
		//###################################################
		//rain 2018-9-21
		//nozzzle turn at 30~90
		Kn_nozzle = (60 + 30.0 * (double)sever_nozzle_sine())/57.3;//遥控器6 967~1815映射0~100度 在换成弧度
		//###################################################

	}


	//printf("\tKy_nozzle = %.6f\t Kn_nozzle = %.6f\n",Ky_nozzle*57.3,Kn_nozzle*57.3);

//%%%%%%%%%%%%%%%%%%%%%%
	if(Kn_nozzle>99.99999/57.3)
	{
		Kn_nozzle=99.99999/57.3;
	}
	if(Kn_nozzle<0.0001)
	{
		Kn_nozzle=0.0001;
	}

	a2_nozzle =  acos((cos(Kn_nozzle/2)-cos(a_nozzle)*cos(a_nozzle))/(sin(a_nozzle)*sin(a_nozzle))) ;//二级喷管角度
	//c2_nozzle = (180-a2_nozzle * 57.3)*2.5*1.1667; //二级舵机输入角度值,1.9412表示转盘更换后的比例
	c2_nozzle[1] = (a2_nozzle * 57.3)*2.5*1.1167; //二级舵机输入角度值,1.9412表示转盘更换后的比例



	//rain 2018-9-20 20:11:51
	//限制二级喷管最大转动速率

	run_dt = hrt_absolute_time()/1000000.0 - run_dt;   //uint second

	c2_nozzle[1] = min(c2_nozzle[0]+160.0*run_dt , c2_nozzle[1]);

	c2_nozzle[1] = max(c2_nozzle[0]-160.0*run_dt, c2_nozzle[1]);

	//更新上次循环的值
	run_dt = hrt_absolute_time()/1000000.0;
	c2_nozzle[0] = c2_nozzle[1];





    if(c2_nozzle[1] < 0)
    { 
        c2_nozzle[1] = 0;
    } 
    else if(c2_nozzle[1] > 502.51)
    {
        c2_nozzle[1] = 502.51;
    }

	//a2_nozzle= (180-c2_nozzle/2.5/1.1667)/57.3; //1.9412表示转盘更换后的比例
	a2_nozzle= (c2_nozzle[1]/2.5/1.1667)/57.3; //1.9412表示转盘更换后的比例
	
    a1_nozzle =  -atan(tan(a2_nozzle/2)*cos(a_nozzle)) + Ky_nozzle  ;  	//一级喷管角度
    //c1_nozzle =  -(90+a1_nozzle * 57.3)*2.55*1.1667;
	c1_nozzle =  -(a1_nozzle * 57.3)*2.55*1.762;

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	chuart.setServoCtrl( c1_nozzle ,  c2_nozzle );
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	//printf("\tc1_nozzle = %.4f\tc2_nozzle = %.4f\n",c1_nozzle,c2_nozzle);

}

//舵机角度阶梯变化函数
float sever_nozzle::sever_nozzle_step()
{
	static int8_t i = 0;
	static uint64_t time_t1 =  hrt_absolute_time();
	uint8_t dir_switch = switch_cmd;
	if((hrt_absolute_time() - time_t1) > STEP_PERIOD)
		{
			time_t1 =  hrt_absolute_time();
			rc2_input = rc2_array[i];

			if(dir_switch == 0x01) 
			{i++;}
			else if(dir_switch == 0x00) 
			{i--;}
				
			
			if(i >= (int8_t)(sizeof(rc2_array)/sizeof(float)))
			{
				i = sizeof(rc2_array)/sizeof(float)-1;
			}
			else if(i < 0)
			{
				i = 0;
			}
				
		}
		//printf("sizeof = %d\n",sizeof(rc2_array)/sizeof(float));

	return rc2_input;
}



//舵机角度阶梯变化函数
float sever_nozzle::sever_nozzle_sine()
{
	static double wt = 0.0;
	static uint64_t time_t2 =  hrt_absolute_time();
	
	uint64_t sin_dt = (hrt_absolute_time()-time_t2);
	if(sin_dt >= SIN_PERIOD)
	{
		time_t2 = hrt_absolute_time();
		sin_dt = SIN_PERIOD;
	}
	
//	wt = (sin_dt/(SIN_PERIOD*1.0) - 0.25) * 2.0 * 3.14;	
	//rc2_input = RC2_INPUT_HMAX * sin(wt);
//	rc2_input = sin(wt) + 1.0;

	//###################################################
	//rain 2018-9-21
	//nozzle turn at 30~90
	wt = (sin_dt/(SIN_PERIOD*1.0)) * 2.0 *3.14;
	rc2_input = sin(wt);
	//###################################################
	//printf("absolute = %d\t",hrt_absolute_time());
	//printf("time_t2 = %d\t",time_t2);
	//printf("sin_dt = %d\t",sin_dt);
	return rc2_input;

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
			SCHED_PRIORITY_DEFAULT + 1,
			2000,
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
	uint8_t i = 0;

	/* do subscriptions */

	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_sever_nozzle_rx_sub = orb_subscribe(ORB_ID(sever_nozzle_rx));

	_task_should_exit = false;


	while (!_task_should_exit) {
		/* only update parameters if they changed */


		//usleep(10000);
		vehicle_manual_poll();
		sever_nozzle_rx_poll();	

		//printf("switch_cmd = 0x%x\n",switch_cmd);
		//calculate nizzle angle
		sever_nozzle_calc();


		//printf("rc2_step = %.4f\n\n",sever_nozzle_step());
		//printf("rc2_sine = %.4f\n\n",(double)sever_nozzle_sine());


		// Advertise/Publish vtol vehicle status
		_sever_nozzle_tx.timestamp = hrt_absolute_time();

		_sever_nozzle_tx.head1 = 0x55;
		_sever_nozzle_tx.head2 = 0xAA;
		_sever_nozzle_tx.timer = 0x00;

		for(i=0; i<_moto_count; i++){
			switch(i)
				{
				case  0 :
					_sever_nozzle_tx.moto_num = 0x01;
					_sever_nozzle_tx.pos = (uint16_t)c1_nozzle * 6.67;
					break;
				case  1 :
					_sever_nozzle_tx.moto_num = 0x02;
					_sever_nozzle_tx.pos = (uint16_t)c2_nozzle[1] * 6.67;
					break;
				default : break;
				}
			_sever_nozzle_tx.checksum = (uint8_t)(((_sever_nozzle_tx.pos >> 8) & 0xff) + (_sever_nozzle_tx.pos & 0xff) + ((_sever_nozzle_tx.timer >> 8) & 0xff) + (_sever_nozzle_tx.timer & 0xff) + (_sever_nozzle_tx.moto_num & 0x03));

			if (_sever_nozzle_tx_pub != nullptr) {
			orb_publish(ORB_ID(sever_nozzle_tx), _sever_nozzle_tx_pub, &_sever_nozzle_tx);
			//printf("orb_publish(ORB_ID(sever_nozzle_tx)\n");
			
			} else {
			_sever_nozzle_tx_pub = orb_advertise(ORB_ID(sever_nozzle_tx), &_sever_nozzle_tx);
			}

		usleep(1500);
		//printf("secer_cpp i = %d\n",i);
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



