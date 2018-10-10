#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <assert.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>


#include <sys/types.h>
#include <stdint.h>
#include <stddef.h>
#include <semaphore.h>
#include <getopt.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>

#include <px4_workqueue.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <px4_time.h>

#include <board_config.h>
#include <parameters/param.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sever_nozzle_rx.h>
#include <uORB/topics/sever_nozzle_rx2.h>
#include <uORB/topics/sever_nozzle_tx.h>

#include <uORB/uORB.h>

#include "sever_nozzle_driver.h"



static bool thread_should_exit = false;
static bool thread_running = false;
static int daemon_task;



 
 // handlers for subscriptions 
 int sever_nozzle_tx_sub = -1;
 orb_advert_t sever_nozzle_rx_pub = NULL;
 orb_advert_t sever_nozzle_rx2_pub = NULL;

// only for publish & subscribe
 //struct sever_nozzle_rx_s sever_rxdata; //定义类型为mytopic_s的结构体变量rd
 struct  sever_nozzle_tx_s sever_txdata;

//数据处理
 unsigned char buf[24];
 //struct Servo_data sever_rxbuf;
struct Servo_data sever_txbuf[2];
 
struct  sever_nozzle_rx_s sever_rxbuf;
//struct  sever_nozzle_tx_s sever_txbuf;





__EXPORT int pi_uart_main(int argc, char *argv[]);
int pi_uart_thread_main(int argc, char *argv[]);
static void usage(const char *reason);




static int uart_init(char * uart_name);
static int set_uart_baudrate(const int fd, unsigned int baud);
int sever_update_rxbuf(int fd);

static int sever_update_txbuf(void);

static void usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: pi_uart {start|stop|status} [param]\n\n");
	exit(1);
}


int uart_init(char * uart_name)
{
	int serial_fd = open(uart_name, O_RDWR | O_NOCTTY);

	if (serial_fd < 0) {
		err(1, "failed to open port: %s", uart_name);
		return false;
	}
	return serial_fd;
}

int set_uart_baudrate(const int fd, unsigned int baud)
{
	int speed;

	switch (baud) {
		case 9600:   speed = B9600;   break;
		case 19200:  speed = B19200;  break;
		case 38400:  speed = B38400;  break;
		case 57600:  speed = B57600;  break;
		case 115200: speed = B115200; break;
		default:{
					warnx("ERR: baudrate: %d\n", baud);
					return -EINVAL;
				}
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	//tcgetattr(fd, &uart_config);

	if ((termios_state = tcgetattr(fd, &uart_config)) < 0) {
		warnx("ERR: %d (tcgetattr)\n", termios_state);
		return false;
	}

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);
	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetispeed)\n", termios_state);
		return false;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		warnx("ERR: %d (cfsetospeed)\n", termios_state);
		return false;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		warnx("ERR: %d (tcsetattr)\n", termios_state);
		return false;
	}

	return true;
}






//error -1, success 1;
int sever_update_rxbuf(int fd)
{

	//读取串口数据
	uint16_t count = 0;
	for(uint i=0; i<sizeof(buf); i++)
	{
		read(fd,&buf[i],1);

		count++;
	}

	if (count > 0) {
		for (uint16_t i = 0;i < count;i++) {
			if ((uint8_t)buf[i] == 0x55 && (uint8_t)buf[i+1] == 0xAA) {
				//checksum
				uint8_t csum = 0;
				for (int8_t j = 2;j < 7;j++)
					csum += (uint8_t) buf[i+j];
				if (csum != (uint8_t) buf[i+7]) {
					printf("failed,count=%d\n",count);
					continue;
				}
				//获取舵机号
				uint8_t tmp = buf[i+6];
				uint8_t num = tmp -1;
				if (num > 1) {
					printf("failed @!\n");
					continue;
				}
				//填充对应舵机的topic数据
				//memcpy(&(sever_rxbuf),(buf+i),sizeof(sever_rxbuf));
				sever_rxbuf.head1 = buf[i];
				sever_rxbuf.head2 = buf[i+1];
				sever_rxbuf.pos = (buf[i+2]) | (buf[i+3] << 8);
				sever_rxbuf.i_main = (buf[i+4]) | (buf[i+5] << 8);
				sever_rxbuf.moto_num = (buf[i+6] & 0x0f);
				sever_rxbuf.checksum = (buf[i+7] & 0xff);
				
				sever_rxbuf.timestamp = hrt_absolute_time();

				break;
			}
		}
		//printf("\tcount = %d\n",count);
		return 1;
	}
	return -1;
}


int sever_update_txbuf(void)
{

	sever_txbuf[0].head1 = sever_txdata.head1;
	sever_txbuf[0].head2 = sever_txdata.head2;
	sever_txbuf[0].pos_l = sever_txdata.pos1 & 0xff;
	sever_txbuf[0].pos_h = (sever_txdata.pos1  & 0xff00) >> 8;
	sever_txbuf[0].i_main_l = sever_txdata.i_main & 0xff;
	sever_txbuf[0].i_main_h = (sever_txdata.i_main & 0xff00) >> 8;
	sever_txbuf[0].num = 0x01;
	sever_txbuf[0].checksum = sever_txdata.checksum1;


	sever_txbuf[1].head1 = sever_txdata.head1;
	sever_txbuf[1].head2 = sever_txdata.head2;
	sever_txbuf[1].pos_l = sever_txdata.pos2 & 0xff;
	sever_txbuf[1].pos_h = (sever_txdata.pos2  & 0xff00) >> 8;
	sever_txbuf[1].i_main_l = sever_txdata.i_main & 0xff;
	sever_txbuf[1].i_main_h = (sever_txdata.i_main & 0xff00) >> 8;
	sever_txbuf[1].num = 0x02;
	sever_txbuf[1].checksum = sever_txdata.checksum2;


	



//	memcpy(&sever_txbuf,&sever_txdata,sizeof(sever_txbuf));
//	memcpy(&sever_txbuf,&sever_txdata,sizeof(sever_nozzle_tx_s));

	return 0;
}


/*
 * TELEM1 : /dev/ttyS1
 * TELEM2 : /dev/ttyS2
 * GPS	  : /dev/ttyS3
 * NSH	  : /dev/ttyS5
 * SERIAL4: /dev/ttyS6
 * N/A	  : /dev/ttyS4
 * IO DEBUG (RX only):/dev/ttyS0
 */

int pi_uart_thread_main(int argc, char *argv[])
{
	warnx("[daemon] starting\n");
	thread_running = true;
	int ret = 0;
	
	//rain 2018-9-20
	//pixhawk serial4-----/dev/ttyS6
	//pixhack serial2-----/dev/ttyS2

	int uart_fd = uart_init("/dev/ttyS6");
	if(false == uart_fd)	
		return -1;
	if(false == set_uart_baudrate(uart_fd,57600)){
		printf("[rain]set_uart_baudrate is failed\n");
		return -1;
	}
	printf("[rain]uart init is successful\n");

	//清空sever_rxbuffer内存空间
	memset(&buf, 0, sizeof(buf));
	memset(&sever_rxbuf, 0, sizeof(sever_rxbuf));
	memset(&sever_txbuf, 0, sizeof(sever_txbuf));

	sever_nozzle_tx_sub = orb_subscribe(ORB_ID(sever_nozzle_tx));

	while(!thread_should_exit){


		//查看数据更新状况，决定是否要清除rx内存

		//读取串口数据
		ret = sever_update_rxbuf(uart_fd);
		if(ret < 0){
			warnx("[rain] recvive dat error");
		}else{
			//publish sever_rxbuf .topic datas
			//发布舵机反馈的消息
			
		//	memcpy(&sever_rxdata,&sever_rxbuf,sizeof(sever_rxbuf));

		//publish sever_nozzle_m2_pos_msg
			
		if((uint8_t)sever_rxbuf.moto_num == 0x02){

			if (sever_nozzle_rx2_pub != NULL) {
			orb_publish(ORB_ID(sever_nozzle_rx2), sever_nozzle_rx2_pub, &sever_rxbuf);
			
			} else {
			sever_nozzle_rx2_pub = orb_advertise(ORB_ID(sever_nozzle_rx2), &sever_rxbuf);
			}

		}else{
		// publish sever_nozzle_rxdata from uart
		
		if (sever_nozzle_rx_pub != NULL) {
		orb_publish(ORB_ID(sever_nozzle_rx), sever_nozzle_rx_pub, &sever_rxbuf);
		
		} else {
		sever_nozzle_rx_pub = orb_advertise(ORB_ID(sever_nozzle_rx), &sever_rxbuf);
		}

		}
			
			//warnx("[rain] publish dat success");
		}

		

		//检查sever_txbuffer .topic是否更新数据
		/* check for changes in sever_nozzle_tx and subscribe tx cmd */
		bool updated;
		
		/* get pilots inputs */
		orb_check(sever_nozzle_tx_sub, &updated);
		
		if (updated) {
			orb_copy(ORB_ID(sever_nozzle_tx), sever_nozzle_tx_sub, &sever_txdata);

			//数据处理		
			sever_update_txbuf();
			//发送串口数据
		}


		
		write(uart_fd,&sever_txbuf[0],sizeof(sever_txbuf[0]));
		write(uart_fd,&sever_txbuf[1],sizeof(sever_txbuf[1]));

		usleep(500);
		//printf("sever_rxbuf\t%x\t%x\t%x\n",sever_rxbuf.head1, sever_rxbuf.head2, sever_rxbuf.checksum);
		//printf("sever_txbuf\t%x\t%x\t%x\n",sever_txbuf.head1, sever_txbuf.pos_l, sever_txbuf.checksum);
		//printf("sever_txdata\t%x\t%x\t%x\n",sever_txdata.head1, sever_txdata.pos, sever_txdata.checksum);

	}
	warnx("[rain]exiting");
	thread_running = false;
	close(uart_fd);

	fflush(stdout);


	return 0;
}



int pi_uart_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("[rain]missing command");
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("[rain]already running\n");
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("pi_uart",
				SCHED_DEFAULT,
				SCHED_PRIORITY_DEFAULT,
				2000,
				pi_uart_thread_main,
				(argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("[rain]running");

		} else {
			warnx("[rain]stopped");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}



