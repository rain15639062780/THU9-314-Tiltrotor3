/**************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file adis16448.cpp
 *
 * Driver for the Analog device ADIS16448 connected via SPI.
 *
 * @author Amir Melzer
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author Lorenz Meier <lm@inf.ethz.ch>
 *
 */

#include <px4_config.h>
#include <ecl/geo/geo.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>

#include <board_config.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

//rain 2018-9-11 08:56:42
//add user topics header
#include <uORB/topics/sensor_gyro_adis16488.h>
#include <uORB/topics/sensor_accel_adis16488.h>
#include <uORB/topics/sensor_mag_adis16488.h>

//rain 2018-9-11 08:55:25
//user topic log enable switch
//1：enable,0:disable
#define USER_TOPIC_LOG_ENABLE	1



#define DIR_READ				0x00
#define DIR_WRITE				0x80

#define ADIS16488_DEVICE_PATH_ACCEL		"/dev/adis16448_accel"
#define ADIS16488_DEVICE_PATH_GYRO		"/dev/adis16448_gyro"
#define ADIS16488_DEVICE_PATH_MAG		"/dev/adis16448_mag"

//rain 2018-8-29 17:00:43
//  ADIS16488  page registers
#define ADIS16488_PAGE_ADDR  	0x00  /* ADIS16488 PAGE ID ADDRESS*/

#define ADIS16488_PAGE0  	0x00  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE1  	0x01  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE2  	0x02  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE3  	0x03  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE4  	0x04  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE5  	0x05  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE6  	0x06  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE7  	0x07  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE8  	0x08  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGE9  	0x09  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGEA  	0x0A  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGEB  	0x0B  /* ADIS16488 PAGE ID */
#define ADIS16488_PAGEC  	0x0C  /* ADIS16488 PAGE ID */


//rain 2018-8-29 17:01:58
//  ADIS16488  page0 registers
#define ADIS16488_TEMP_OUT  	0x0E  /* ADIS16488 temperature output */
#define ADIS16488_XGYRO_OUT 	0x12  /* X-axis gyroscope output */
#define ADIS16488_YGYRO_OUT 	0x16  /* Y-axis gyroscope output */
#define ADIS16488_ZGYRO_OUT 	0x1A  /* Z-axis gyroscope output */
#define ADIS16488_XACCL_OUT 	0x1E  /* X-axis accelerometer output */
#define ADIS16488_YACCL_OUT 	0x22  /* Y-axis accelerometer output */
#define ADIS16488_ZACCL_OUT 	0x26  /* Z-axis accelerometer output */
#define ADIS16488_XMAGN_OUT 	0x28  /* X-axis magnetometer measurement */
#define ADIS16488_YMAGN_OUT 	0x2A  /* Y-axis magnetometer measurement */
#define ADIS16488_ZMAGN_OUT 	0x2C  /* Z-axis magnetometer measurement */
#define ADIS16488_BARO_OUT  	0x30  /* Barometric pressure output */

//rain 2018-8-29 17:01:58
//  ADIS16488  page0 registers
#define ADIS16488_PRODUCT_ID 	0x7E  /* Product identifier */
#define ADIS16488_Product	0x4068/* Product ID Description for ADIS16448 */


//rain 2018-8-29 17:01:58
//  ADIS16488  page3 registers
#define ADIS16488_GLOB_CMD  	0x02  /* System command */
#define ADIS16488_FNCTIO_CTRL   0x06  /* Control, I/O pins, functional definitions */
#define ADIS16488_GPIO_CTRL 	0x08  /* Auxiliary digital input/output control */
#define ADIS16488_DEC_RATE      0X0C  /* output sample rate decimation */
#define ADIS16488_FILTER_BNK_0  0x16   
#define ADIS16488_FILTER_BNK_1  0x18 

#define ADIS16488_FILTER_A 0x4924
#define ADIS16488_FILTER_B 0x5B6D
#define ADIS16488_FILTER_C 0x6DB6
#define ADIS16488_FILTER_D 0x7FFF

#define ADIS16488_FILTER_EN_A 0x04
#define ADIS16488_FILTER_EN_B 0x05
#define ADIS16488_FILTER_EN_C 0x06
#define ADIS16488_FILTER_EN_D 0x07


//rain 2018-8-29 17:01:58
//  ADIS16488  page4 registers
#define ADIS16488_SERIAL_NUMBER 0x20  /* Serial number, lot specific */

//rain 2018-9-6 14:04:22
//adis16488 irq_pin
#define ADIS16488_DRDY_IRQ_GPIO  (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz|GPIO_PORTC|GPIO_PIN14)

#define ADIS16488_DRDY          px4_arch_gpioread(ADIS16488_DRDY_IRQ_GPIO)



/* Calibration parameters */
#define ADIS16448_XGYRO_OFF 	0x1A  /* X-axis gyroscope bias offset factor */
#define ADIS16448_YGYRO_OFF 	0x1C  /* Y-axis gyroscope bias offset factor */
#define ADIS16448_ZGYRO_OFF 	0x1E  /* Z-axis gyroscope bias offset factor */
#define ADIS16448_XACCL_OFF 	0x20  /* X-axis acceleration bias offset factor */
#define ADIS16448_YACCL_OFF 	0x22  /* Y-axis acceleration bias offset factor */
#define ADIS16448_ZACCL_OFF 	0x24  /* Z-axis acceleration bias offset factor */
#define ADIS16448_XMAGN_HIC 	0x26  /* X-axis magnetometer, hard-iron factor */
#define ADIS16448_YMAGN_HIC 	0x28  /* Y-axis magnetometer, hard-iron factor */
#define ADIS16448_ZMAGN_HIC 	0x2A  /* Z-axis magnetometer, hard-iron factor */
#define ADIS16448_XMAGN_SIC	0x2C  /* X-axis magnetometer, soft-iron factor */
#define ADIS16448_YMAGN_SIC 	0x2E  /* Y-axis magnetometer, soft-iron factor */
#define ADIS16448_ZMAGN_SIC 	0x30  /* Z-axis magnetometer, soft-iron factor */

#define ADIS16448_GPIO_CTRL 	0x32  /* Auxiliary digital input/output control */
#define ADIS16448_MSC_CTRL  	0x34  /* Miscellaneous control */
#define ADIS16448_SMPL_PRD  	0x36  /* Internal sample period (rate) control */
#define ADIS16448_SENS_AVG  	0x38  /* Dynamic range and digital filter control */
#define ADIS16448_SLP_CNT   	0x3A  /* Sleep mode control */
#define ADIS16448_DIAG_STAT 	0x3C  /* System status */

/* Alarm functions */
#define ADIS16448_GLOB_CMD  	0x3E  /* System command */
#define ADIS16448_ALM_MAG1  	0x40  /* Alarm 1 amplitude threshold */
#define ADIS16448_ALM_MAG2  	0x42  /* Alarm 2 amplitude threshold */
#define ADIS16448_ALM_SMPL1 	0x44  /* Alarm 1 sample size */
#define ADIS16448_ALM_SMPL2 	0x46  /* Alarm 2 sample size */
#define ADIS16448_ALM_CTRL  	0x48  /* Alarm control */

#define ADIS16334_LOT_ID1   	0x52  /* Lot identification code 1 */
#define ADIS16334_LOT_ID2   	0x54  /* Lot identification code 2 */
#define ADIS16448_PRODUCT_ID 	0x56  /* Product identifier */
#define ADIS16334_SERIAL_NUMBER 0x58  /* Serial number, lot specific */

#define ADIS16448_Product	0x4040/* Product ID Description for ADIS16448 */

#define BITS_SMPL_PRD_NO_TAP_CFG 	(0<<8)
#define BITS_SMPL_PRD_2_TAP_CFG	 	(1<<8)
#define BITS_SMPL_PRD_4_TAP_CFG	 	(2<<8)
#define BITS_SMPL_PRD_8_TAP_CFG	 	(3<<8)
#define BITS_SMPL_PRD_16_TAP_CFG	(4<<8)

#define BITS_GYRO_DYN_RANGE_1000_CFG (4<<8)
#define BITS_GYRO_DYN_RANGE_500_CFG	 (2<<8)
#define BITS_GYRO_DYN_RANGE_250_CFG	 (1<<8)




//rain 2018-9-6 20:35:33
//adis16488 register

//rain 2018-9-6
//modify spi speed to 11Mhz
#define SPI_BUS_SPEED								11*1000*1000
#define T_STALL										6

//rain 2018-9-6

#define ADIS16488_ACCEL_GYRO_MAX_SAMPLE_RATE   2460
#define ADIS16488_ACCEL_GYRO_DEFAULT_SAMPLE_RATE   410

//ADIS16488 OUTPUT RATE
#define ADIS16488_ACCEL_MAX_RATE              1000
#define ADIS16488_ACCEL_DEFAULT_RATE			1000	//modify accel speed to 800hz0
#define ADIS16488_ACCEL_OUTPUT_RATE             280//280//400//280//410//1000//410
#define ADIS16488_ACCEL_DEFAULT_DRIVER_FILTER_FREQ	30//25//20//15//10//30


#define ADIS16488_GYRO_MAX_RATE               1000
#define ADIS16488_GYRO_DEFAULT_RATE					1000
#define ADIS16488_GYRO_OUTPUT_RATE             ADIS16488_ACCEL_OUTPUT_RATE//1000//410
#define ADIS16488_GYRO_DEFAULT_DRIVER_FILTER_FREQ	30//25//20//15//5//10//30

#define ADIS16488_MAG_MAX_SAMPLE_RATE						102.5f
#define ADIS16488_MAG_MAX_RATE                102
#define ADIS16488_MAG_DEFAULT_RATE		100//1000	//  100
#define ADIS16488_MAG_OUTPUT_RATE            100// ADIS16488_ACCEL_OUTPUT_RATE
#define ADIS16488_MAG_DEFAULT_DRIVER_FILTER_FREQ	30//25//20//15//10//30



//传感器灵敏度配置
//满量程最大值
#define TEMPERATUREINITIALSENSITIVITY				0.00565f  //+25℃
#define ACCELINITIALSENSITIVITY						0.0008f  //1 lsb = 0.8mg
#define GYROINITIALSENSITIVITY						0.02    //暂时不确定  1 lsb = 0.02°/sec  
#define MAGINITIALSENSITIVITY						(1.0f / 10000.0f)  //1 lsb = 0.1 mgass


//传感器量程配置
#define ACCELDYNAMICRANGE							18.0f
#define GYRODYNAMICRANGE							450.0f
#define MAGDYNAMICRANGE								3.2767f

//是否启用 LowPassFilter2p.cpp  
#define FW_FILTER				true//false //true
//是否使用IMU内置滤波器库
#define ADIS16448_CORE_EILTER_ENABLE            false//true //true  //false
//使用的 IMU内置滤波器库
#define ADIS16448_CORE_EILTER_USE               ADIS16488_FILTER_B

#define ADIS16488_ACCEL_TIMER_REDUCTION				0//400
#define ADIS16488_GYRO_TIMER_REDUCTION				0//200

class ADIS16488_gyro;
class ADIS16488_mag;
//class ADIS16488_baro;


class ADIS16488 : public device::SPI
{
public:
	ADIS16488(int bus, const char *path_accel, const char *path_gyro, const char *path_mag, uint32_t device,
		  enum Rotation rotation);
	virtual ~ADIS16488();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver and sensor.
	 */
	void			print_info();

	void 			print_calibration_data();

	//rain 2018-9-6
	//adis16488 dio2 数据就绪引脚
	bool			adis16488_drdy_status;    
protected:
	virtual int		probe();

	friend class ADIS16488_gyro;
	friend class ADIS16488_mag;
//	friend class ADIS16488_baro;


	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual ssize_t		mag_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		mag_ioctl(struct file *filp, int cmd, unsigned long arg);


private:
	ADIS16488_gyro		*_gyro;
	ADIS16488_mag		*_mag;
//	ADIS16488_baro		*_baro;

	uint16_t			_product;	/** product code */

	//rain 2018-9-6 21:07:18
	//start()开启定时器回调函数使用
	struct hrt_call		_accel_call;
	struct hrt_call		_gyro_call;
	struct hrt_call		_mag_call;
//	struct hrt_call		_baro_call;

	unsigned		_accel_call_interval;
	unsigned		_gyro_call_interval;
	unsigned		_mag_call_interval;
//	unsigned		_call_baro_interval;	




	ringbuffer::RingBuffer			*_gyro_reports;
	struct gyro_calibration_s	_gyro_scale;
	float				_gyro_range_scale;
	float				_gyro_range_rad_s;

	ringbuffer::RingBuffer			*_accel_reports;
	struct accel_calibration_s	_accel_scale;
	float				_accel_range_scale;
	float				_accel_range_m_s2;
	


	ringbuffer::RingBuffer			*_mag_reports;
	struct mag_calibration_s	_mag_scale;
	float				_mag_range_scale;
	float				_mag_range_mgauss;


	orb_advert_t		_accel_topic;
	orb_advert_t        _accel_topic_adis16488;
	int					_accel_orb_class_instance;
	int					_accel_class_instance;


	unsigned			_gyro_sample_rate;
	unsigned			_accel_sample_rate;
	unsigned			_mag_sample_rate;
	
	perf_counter_t		_gyro_reads;
	perf_counter_t		_accel_reads;
	perf_counter_t		_mag_reads;
	
	perf_counter_t		_gyro_sample_perf;
	perf_counter_t		_accel_sample_perf;
	perf_counter_t		_mag_sample_perf;
	
	perf_counter_t		_gyro_bad_transfers;
	perf_counter_t		_accel_bad_transfers;
	perf_counter_t		_mag_bad_transfers;


	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;
	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_mag_filter_x;
	math::LowPassFilter2p	_mag_filter_y;
	math::LowPassFilter2p	_mag_filter_z;

	Integrator			_accel_int;
	Integrator			_gyro_int;
	Integrator			_mag_int;

	//rain 2018-9-8 15:12:44
	// values used to
	float			_last_accel[3];
	uint8_t			_constant_accel_count;
	float			_last_gyro[3];
	uint8_t			_constant_gyro_count;

	// last temperature value
	int16_t			_last_temperature_raw;
	float			_last_temperature;

	enum Rotation		_rotation;


	/**
	 * Start automatic measurement.
	 */
	void		start();

	/**
	 * Stop automatic measurement.
	 */
	void		stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Static trampoline for the mag because it runs at a lower rate
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		gyro_measure_trampoline(void *arg);

	/**
	 * Static trampoline for the mag because it runs at a lower rate
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		mag_measure_trampoline(void *arg);



	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int			measure();

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int			gyro_measure();

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	int			mag_measure();


	/**
	 * Read a register from the ADIS16448
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint16_t		read_reg16(unsigned reg);

	/**
	 * Write a register in the ADIS16448
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void 			write_reg16(unsigned reg, uint16_t value);

	//rain 2018-9-6 21:21:22
	//add check register value
	/**
	 * Check a register in the ADIS16448
	 *
	 * @param reg		The register to write.
	 * @param value		The set value to confirm.
	 */
	int 			check_reg16(unsigned reg, uint16_t value);

	/**
	 * Modify a register in the ADIS16448
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits);

	/**
	 * Swap a 16-bit value read from the ADIS16448 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * convert 12 bit integer format to int16.
	 */
	int16_t			convert12BitToINT16(int16_t word);

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			gyro_self_test();

		/**
	 * mag self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			mag_self_test();

	/*
	  set low pass filter frequency BOTH ACCEL AND GYRO
	 */
	void _accel_gyro_set_dlpf_filter(uint16_t frequency_hz);




	/*
	  set IMU to factory default
	 */
	void _set_factory_default();

	/*
	  set accel & gyro sample rate (approximate) - 1kHz to 5Hz
	  register DEC_RATE   page3 0x0c
	*/
	void _accel_gyro_set_sample_rate(uint16_t desired_sample_rate_hz);



	/*
	  set the gyroscope dynamic range
	*/
	void _set_gyro_dyn_range(float desired_gyro_dyn_range);
	

	ADIS16488(const ADIS16488 &);
	ADIS16488 operator=(const ADIS16488 &);
};

/**
 * Helper class implementing the gyro driver node.
 */
class ADIS16488_gyro : public device::CDev
{
public:
	ADIS16488_gyro(ADIS16488 *parent, const char *path);
	virtual ~ADIS16488_gyro();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class ADIS16488;

	void			parent_poll_notify();
private:
	ADIS16488			*_parent;
	orb_advert_t		_gyro_topic;
	orb_advert_t       _gyro_topic_adis16488;
	
	int					_gyro_orb_class_instance;
	int					_gyro_class_instance;

	void				measure();

	void				measure_trampoline(void *arg);

	/* do not allow to copy this class due to pointer data members */
	ADIS16488_gyro(const ADIS16488_gyro &);
	ADIS16488_gyro operator=(const ADIS16488_gyro &);

};
/**
 * Helper class implementing the mag driver node.
 */
class ADIS16488_mag : public device::CDev
{
public:
	ADIS16488_mag(ADIS16488 *parent, const char *path);
	virtual ~ADIS16488_mag();
	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class ADIS16488;

	void			parent_poll_notify();
private:
	ADIS16488			*_parent;
	orb_advert_t		_mag_topic;
	orb_advert_t		_mag_topic_adis16488;
	int					_mag_orb_class_instance;
	int					_mag_class_instance;

	void				measure();

	void				measure_trampoline(void *arg);


	/* do not allow to copy this class due to pointer data members */
	ADIS16488_mag(const ADIS16488_mag &);
	ADIS16488_mag operator=(const ADIS16488_mag &);

};
/** driver 'main' command */
extern "C" { __EXPORT int adis16448_main(int argc, char *argv[]); }

ADIS16488::ADIS16488(int bus, const char *path_accel, const char *path_gyro, const char *path_mag, uint32_t device,
		     enum Rotation rotation) :
	SPI("ADIS16488", path_accel, bus, device, SPIDEV_MODE3, SPI_BUS_SPEED),
	_gyro(new ADIS16488_gyro(this, path_gyro)),
	_mag(new ADIS16488_mag(this, path_mag)),
	_product(0),
	_accel_call{},
	_gyro_call{},
	_mag_call{},
	_accel_call_interval(0),
	_gyro_call_interval(0),
	_mag_call_interval(0),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_mag_reports(nullptr),
	_mag_scale{},
	_mag_range_scale(0.0f),
	_mag_range_mgauss(0.0f),	
	_accel_topic(nullptr),
	_accel_topic_adis16488(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),

	_gyro_sample_rate(1000),	
	_accel_sample_rate(1000),	
	_mag_sample_rate(1000),	
	/* Init sampling frequency set to 100Hz */
	_gyro_reads(perf_alloc(PC_COUNT, "adis16488_gyro_read")),
	_accel_reads(perf_alloc(PC_COUNT, "adis16488_accel_read")),
	_mag_reads(perf_alloc(PC_COUNT, "adis16488_mag_read")),
	_gyro_sample_perf(perf_alloc(PC_ELAPSED, "adis16488_gyro_read")),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "adis16488_accel_read")),
	_mag_sample_perf(perf_alloc(PC_ELAPSED, "adis16488_mag_read")),
	_gyro_bad_transfers(perf_alloc(PC_COUNT, "adis16488_gyro_bad_transfers")),
	_accel_bad_transfers(perf_alloc(PC_COUNT, "adis16488_accelbad_transfers")),
	_mag_bad_transfers(perf_alloc(PC_COUNT, "adis16488_mag_bad_transfers")),

	_gyro_filter_x(ADIS16488_GYRO_DEFAULT_RATE, ADIS16488_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(ADIS16488_GYRO_DEFAULT_RATE, ADIS16488_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(ADIS16488_GYRO_DEFAULT_RATE, ADIS16488_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_x(ADIS16488_ACCEL_DEFAULT_RATE, ADIS16488_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(ADIS16488_ACCEL_DEFAULT_RATE, ADIS16488_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(ADIS16488_ACCEL_DEFAULT_RATE, ADIS16488_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_mag_filter_x(ADIS16488_MAG_DEFAULT_RATE, ADIS16488_MAG_DEFAULT_DRIVER_FILTER_FREQ),
	_mag_filter_y(ADIS16488_MAG_DEFAULT_RATE, ADIS16488_MAG_DEFAULT_DRIVER_FILTER_FREQ),
	_mag_filter_z(ADIS16488_MAG_DEFAULT_RATE, ADIS16488_MAG_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / ADIS16488_ACCEL_OUTPUT_RATE, true),
	_gyro_int(1000000 / ADIS16488_GYRO_OUTPUT_RATE, true),
	_mag_int(1000000 / ADIS16488_MAG_OUTPUT_RATE, true),
	_constant_accel_count(0),
	_constant_gyro_count(0),
	_last_temperature_raw(0),
	_last_temperature(0),
	_rotation(rotation)
	
{
	// disable debug() calls
	_debug_enabled = true;

	//rain 2018-9-6 14:17:02
	//init adis16448 gpio irq pin	
	stm32_configgpio(ADIS16488_DRDY_IRQ_GPIO);

	warnx("rain:ADIS16488::ADIS16488 start");

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_ADIS16488;

	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_ADIS16488;

	_mag->_device_id.devid = _device_id.devid;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_ADIS16488;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default mag scale factors
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;

	memset(&_accel_call, 0, sizeof(_accel_call));
	memset(&_gyro_call, 0, sizeof(_gyro_call));
	memset(&_mag_call, 0, sizeof(_mag_call));

	warnx("rain: ADIS16448::ADIS16488 exit");
}

ADIS16488::~ADIS16488()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;
	delete _mag;

	/* free any existing reports */
	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_accel_sample_perf);
	perf_free(_gyro_sample_perf);
	perf_free(_mag_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_mag_reads);
	perf_free(_accel_bad_transfers);
	perf_free(_gyro_bad_transfers);
	perf_free(_mag_bad_transfers);
}

int
ADIS16488::init()
{
	int ret;
	warnx("rain:ADIS16488::init start");


	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}



	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	/* allocate basic report buffers */
	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

	if (_gyro_reports == nullptr) {
		goto out;
	}

	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

	if (_accel_reports == nullptr) {
		goto out;
	}

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr) {
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

	/* Initialize offsets and scales */
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_mag_scale.x_offset   = 0;
	_mag_scale.x_scale    = 1.0f;
	_mag_scale.y_offset   = 0;
	_mag_scale.y_scale    = 1.0f;
	_mag_scale.z_offset   = 0;
	_mag_scale.z_scale    = 1.0f;



	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	/* do CDev init for the gyro device node, keep it optional */
	ret = _mag->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}


	/* fetch an initial set of measurements for advertisement */
	ret = measure();
	if(ret != OK){
		warnx("measure() failed");
	}
	warnx("have pass accel measure()");

	ret = gyro_measure();
	if(ret != OK){
		warnx("gyro_measure() failed");
	}
	warnx("have pass gyro measure()");

	ret = mag_measure();
		if(ret != OK){
			warnx("mag measure() failed");
		}
	warnx("have pass mag measure()");

	/* advertise sensor topic, measure manually to initialize valid report */
/**/
	struct accel_report arp;
	_accel_reports->get(&arp);
/**/

	/* measurement will have generated a report, publish */
/**/	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, ORB_PRIO_MAX);

	if (_accel_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	struct gyro_report grp;

	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance, ORB_PRIO_MAX);

	if (_gyro->_gyro_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

	struct mag_report mrp;

	_mag_reports->get(&mrp);

	_mag->_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
					       &_mag->_mag_orb_class_instance, ORB_PRIO_MAX);

	if (_mag->_mag_topic == nullptr) {
		warnx("ADVERT FAIL");
	}
	
	//add user topic advertise

	if(1 == USER_TOPIC_LOG_ENABLE){
		_gyro->_gyro_topic_adis16488 = orb_advertise(ORB_ID(sensor_gyro_adis16488), &grp);

		if (_gyro->_gyro_topic_adis16488 == nullptr) {
			warnx("_gyro_topicadis16488 ADVERT FAIL");
		}	
		_accel_topic_adis16488 = orb_advertise(ORB_ID(sensor_accel_adis16488), &arp);

		if (_accel_topic_adis16488 == nullptr) {
			warnx("_accel_topicadis16488 ADVERT FAIL");
		}	
		_mag->_mag_topic_adis16488 = orb_advertise(ORB_ID(sensor_mag_adis16488), &mrp);

		if (_mag->_mag_topic_adis16488 == nullptr) {
			warnx("_mag_topicadis16488 ADVERT FAIL");
		}	

	}	
	

/**/

  warnx("rain: ADIS16488::init exit");

out:
	return ret;
}

int ADIS16488::reset()
{

	//rain 2018-8-30 10:53:49

	//1- Set IMU sample rate 
	//_set_sample_rate(_sample_rate);
	 //设置IMUaccel & gyro输出速率
	//输出速率设为2460/(122+1)=200Hz
	//write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE3);
	//write_reg16(ADIS16488_DEC_RATE, 0x0B);
	_accel_gyro_set_sample_rate(ADIS16488_ACCEL_GYRO_DEFAULT_SAMPLE_RATE); 
	//磁力计和气压计的输出更新速率不需设置
	//mag_out_sample_rate = 102.5 sps
	//baro_out_samole_rate = 51.25 sps

	//2- Set gyroscope scale to default value 
	//设置gyro的分辨率&阈值
	_set_gyro_dyn_range(GYROINITIALSENSITIVITY);   
	//设置accel的分辨率&阈值
	_accel_range_scale = (float) (CONSTANTS_ONE_G * ACCELINITIALSENSITIVITY);  
	_accel_range_m_s2  = (float) (CONSTANTS_ONE_G * ACCELDYNAMICRANGE);
	 //设置mag的分辨率&阈值
	_mag_range_scale   = (float) MAGINITIALSENSITIVITY; 
	_mag_range_mgauss  = (float) MAGDYNAMICRANGE;


	//3- Set digital FIR filter tap 
	//_accel_gyro_set_dlpf_filter(BITS_FIR_16_TAP_CFG);

	//rain 2018-10-17
	//add adis16488 (自带)滤波器库使能
	if(ADIS16448_CORE_EILTER_ENABLE)
	{
		uint filer_data = 0x00;
		//            GYRO-X | GYRO-Y  | GYRO-Z  | ACC-X   | ACC-Y    | NONE
		//--------------------------------------------------------------------
		//filer_data = 0X04<<0 | 0X04<<3 | 0X04<<6 | 0X04<<9 | 0X04<<12 | 0<<15;
/*
		filer_data =  ADIS16488_FILTER_EN_B << 0 | ADIS16488_FILTER_EN_B << 3   \
			    | ADIS16488_FILTER_EN_B << 6 | ADIS16488_FILTER_EN_B << 9   \
			    | ADIS16488_FILTER_EN_B << 12 | 0X00 << 15;
*/

		filer_data =  ADIS16448_CORE_EILTER_USE << 0 | ADIS16448_CORE_EILTER_USE << 3   \
			    | ADIS16448_CORE_EILTER_USE << 6 | ADIS16448_CORE_EILTER_USE << 9   \
			    | ADIS16448_CORE_EILTER_USE << 12 | 0X00 << 15;		
		write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE3);
		write_reg16(ADIS16488_FILTER_BNK_0, filer_data);

		//            ACC-Z  | MAG-X   | MAG-Y   | MAG-Z   | NONE
		//-------------------------------------------------------------
		//filer_data = 0X04<<0 | 0X04<<3 | 0X04<<6 | 0X04<<9 | 0X00<<12 ;
		/*
		filer_data =  ADIS16488_FILTER_EN_B << 0 | ADIS16488_FILTER_EN_B << 3   \
			    | ADIS16488_FILTER_EN_B << 6 | ADIS16488_FILTER_EN_B << 9   \
			    | 0X00 << 12;
		 */
		filer_data =  ADIS16448_CORE_EILTER_USE << 0 | ADIS16448_CORE_EILTER_USE << 3   \
			    | ADIS16448_CORE_EILTER_USE << 6 | ADIS16448_CORE_EILTER_USE << 9   \
			    | 0X00 << 12;

		write_reg16(ADIS16488_FILTER_BNK_1, filer_data);
		write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE0);

	}

	

	/* settling time */
	up_udelay(50000);
	warnx("class ADIS16448::reset() running");
	return OK;

}

int
ADIS16488::probe()
{
	uint16_t serial_number;
	warnx("rain:ADIS16488::probe start");

	/* retry 5 time to get the ADIS16448 PRODUCT ID number */
	for (int i = 0; i < 5; i++) {
		
		//rain 2018-8-29 17:41:52
		/* recognize product ID */
		write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE0);
	        _product = read_reg16(ADIS16488_PRODUCT_ID);

		if (_product != 0) {
			break;
		}
	}

	/* recognize product serial number */
	//rain 2018-8-29 17:41:52
	serial_number = (read_reg16(ADIS16488_SERIAL_NUMBER) & 0xffff);

	/* verify product ID */
	switch (_product) {
	//rain 2018-8-29 18:04:48
	case ADIS16488_Product:
		DEVICE_DEBUG("ADIS16488 is detected ID: 0x%02x, Serial: 0x%02x", _product, serial_number);

		warnx("rain: ADIS16488::probe exit");
	
		//工厂默认指定DIO2为正极性数据就绪信号
		return OK;

	}

	DEVICE_DEBUG("unexpected ID 0x%02x", _product);

	return -EIO;
}

/* set sample rate for both accel and gyro */
void
ADIS16488::_accel_gyro_set_sample_rate(uint16_t desired_sample_rate_hz)
{
	uint16_t smpl_prd = 0;

	smpl_prd = ADIS16488_ACCEL_GYRO_MAX_SAMPLE_RATE / desired_sample_rate_hz;
	smpl_prd = smpl_prd - 1;

	write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE3);
	write_reg16(ADIS16488_DEC_RATE, smpl_prd);

	if ((read_reg16(ADIS16488_DEC_RATE) & 0x00ff) != smpl_prd) {
		DEVICE_DEBUG("failed to set IMU sample rate");
	}

	write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE0);

}


/* set the DLPF FIR filter tap. This affects both accelerometer and gyroscope. */
void
ADIS16488::_accel_gyro_set_dlpf_filter(uint16_t desired_filter_tap)
{

	//rain 2018-9-6 22:31:49
	//采用imu出厂设置参数
/*
	modify_reg16(ADIS16448_SENS_AVG, 0x0007, desired_filter_tap);

	// * Verify data write on the IMU * /

	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0007) != desired_filter_tap) {
		DEVICE_DEBUG("failed to set IMU filter");
	}
*/

}

/* set IMU to factory defaults. */
void
ADIS16488::_set_factory_default()
{
	//rain 2018-9-6 22:39:07
	//set page3->GLOB_CMD bit6 = 1
	write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE3);
	write_reg16(ADIS16488_GLOB_CMD, 0x40);
	write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE0);
}


/* set the gyroscope dynamic range */
void
ADIS16488::_set_gyro_dyn_range(float desired_gyro_dyn_range)
{
	//rain 2018-9-9 11:30:44
	//设置gyro分辨率值（弧度每秒）
	_gyro_range_scale  = (float)(desired_gyro_dyn_range * 1.0f / 180.0f * M_PI_F);
	_gyro_range_rad_s = (float)(GYRODYNAMICRANGE / 180.0f * M_PI_F);

}


ssize_t
ADIS16488::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);
	warnx("rain: ADIS16488::read start");

	/* buffer must be large enough */
	if (count < 1) {
		warnx("rain:count <1 exit");		
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_accel_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty()) {
		warnx("rain:_accel_reports->empty() exit");		
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	warnx("rain: ADIS16448::read exit");
	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));
}

int
ADIS16488::self_test()
{

	//rain 2018-9-6 22:49:00
	//非关键函数，暂不填充
	/*
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	// * return 0 on success, 1 else * /
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
	*/
	return 0;
}

int
ADIS16488::accel_self_test()
{
	if (self_test()) {
		return 1;
	}

	return 0;
}

int
ADIS16488::gyro_self_test()
{
	if (self_test()) {
		return 1;
	}

	return 0;
}

int
ADIS16488::mag_self_test()
{
	if (self_test()) {
		return 1;
	}

	return 0;
}


ssize_t
ADIS16488::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(gyro_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_gyro_call_interval == 0) {
		_gyro_reports->flush();
		gyro_measure();
	}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(gyro_report));
}

ssize_t
ADIS16488::mag_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(mag_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_mag_call_interval == 0) {
		_mag_reports->flush();
		mag_measure();
	}

	/* if no data, error (we could block here) */
	if (_mag_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_mag_reads);

	/* copy reports out of our buffer to the caller */
	mag_report *mrp = reinterpret_cast<mag_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_mag_reports->get(mrp)) {
			break;
		}

		transferred++;
		mrp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(mag_report));
}



//just for accel
int
ADIS16488::ioctl(struct file *filp, int cmd, unsigned long arg)
{
        //warnx("execute ADIS16448::ioctl arg");
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_accel_call_interval = 0;	
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			//rain 2018-9-2 17:37:51
			//根据传感器手册进行设置最大读取速率
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16488_ACCEL_MAX_RATE);

			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16488_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_accel_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;


					/* check against maximum sane rate */
					//rain 2018-9-6
					//modify ticks range 1000 to 500
					if (ticks < 500) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1000000.0f / ticks;
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
		
					//rain 2018-9-2 17:41:51
					//设置调用周期&调用间隔
					_accel_call_interval = ticks;
					_accel_call.period = _accel_call_interval - ADIS16488_ACCEL_TIMER_REDUCTION;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}
					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_accel_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _accel_call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_accel_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCRESET:
	warnx("execute ADIS16448::ioctl SENSORIOCRESET");
	return reset();

	case ACCELIOCGSAMPLERATE:
		return _accel_sample_rate;

	case ACCELIOCSSAMPLERATE:
		_accel_gyro_set_sample_rate(arg);
		return OK;

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		return -EINVAL;

	case ACCELIOCGRANGE:
		return (unsigned long)((_accel_range_m_s2) / CONSTANTS_ONE_G + 0.5f);

	//rain 2018-10-21
	//firmware 1.8.0 already delete xxxSELFTEST
	//case ACCELIOCSELFTEST:
	//	return accel_self_test();

	case ACCELIOCTYPE:
		return (ADIS16488_Product);

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
ADIS16488::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
		case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_gyro_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			//rain 2018-9-2 17:37:51
			//根据传感器手册进行设置最大读取速率
			case SENSOR_POLLRATE_MAX:
				return gyro_ioctl(filp, SENSORIOCSPOLLRATE, ADIS16488_GYRO_MAX_RATE);

			case SENSOR_POLLRATE_DEFAULT:
				return gyro_ioctl(filp, SENSORIOCSPOLLRATE, ADIS16488_GYRO_DEFAULT_RATE);


			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_gyro_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 500) {
						return -EINVAL;
					}

					// adjust filters
					float sample_rate = 1000000.0f / ticks;

					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);


					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */

					_gyro_call_interval = ticks;
					_gyro_call.period = _gyro_call_interval - ADIS16488_GYRO_TIMER_REDUCTION;
					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_gyro_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case GYROIOCGSAMPLERATE:
		return _gyro_sample_rate;

	case GYROIOCSSAMPLERATE:
		_accel_gyro_set_sample_rate(arg);
		return OK;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_calibration_s *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		_set_gyro_dyn_range(arg);
		return OK;

	case GYROIOCGRANGE:
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	//rain 2018-10-21
	//firmware 1.8.0 already delete xxxSELFTEST
	//case GYROIOCSELFTEST:
	//	return gyro_self_test();

	case GYROIOCTYPE:
		return (ADIS16488_Product);

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
ADIS16488::mag_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE: {
		switch (arg) {
	/* these are shared with the accel side */
		/* switching to manual polling */
		case SENSOR_POLLRATE_MANUAL:
			stop();
			_mag_call_interval = 0;
			return OK;
	
		/* external signalling not supported */
		case SENSOR_POLLRATE_EXTERNAL:
	
		/* zero would be bad */
		case 0:
			return -EINVAL;
	
		/* set default/max polling rate */
		//rain 2018-9-2 17:37:51
		//根据传感器手册进行设置最大读取速率
		case SENSOR_POLLRATE_MAX:
			return mag_ioctl(filp, SENSORIOCSPOLLRATE, ADIS16488_MAG_MAX_RATE);
	
		case SENSOR_POLLRATE_DEFAULT:
			return mag_ioctl(filp, SENSORIOCSPOLLRATE, ADIS16488_MAG_DEFAULT_RATE);
	
	
		/* adjust to a legal polling interval in Hz */
		default: {
				/* do we need to start internal polling? */
				bool want_start = (_mag_call_interval == 0);
	
				/* convert hz to hrt interval via microseconds */
				unsigned ticks = 1000000 / arg;
	
				/* check against maximum sane rate */
				if (ticks < 1000) {
					return -EINVAL;
				}
	
				// adjust filters
				float sample_rate = 1000000.0f / ticks;
	
				float cutoff_freq_hz_mag = _mag_filter_x.get_cutoff_freq();
				_mag_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);
				_mag_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);
				_mag_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);

	
	
				/* update interval for next measurement */
				/* XXX this is a bit shady, but no other way to adjust... */
				_mag_call.period = _mag_call_interval = ticks;
	
				/* if we need to start the poll state machine, do it */
				if (want_start) {
					start();
				}
	
				return OK;
			}
		}

		}
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_mag_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case MAGIOCGSAMPLERATE:
		return _mag_sample_rate;

	case MAGIOCSSAMPLERATE:
		//_set_sample_rate(arg);
		return OK;

	case MAGIOCSSCALE:
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_calibration_s *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE:
		/* copy scale out */
		memcpy((struct mag_calibration_s *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCSRANGE:
		return -EINVAL;

	case MAGIOCGRANGE:
		return (unsigned long)(_mag_range_mgauss);

	//rain 2018-10-21
	//firmware 1.8.0 already delete xxxSELFTEST
	//case MAGIOCSELFTEST:
	//	return OK;

	case MAGIOCTYPE:
		return (ADIS16488_Product);

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}

}

uint16_t
ADIS16488::read_reg16(unsigned reg)
{
	uint16_t cmd[1];

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;
	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(nullptr, cmd, 1);
	up_udelay(T_STALL);

	return cmd[0];
}

void
ADIS16488::write_reg16(unsigned reg, uint16_t value)
{
	//rain 2018-9-5 08:02:29
	//写寄存器
	/**/
	//adis16448 两个SPI时序
	uint16_t	cmd[2];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	up_udelay(T_STALL);
	 /**/
/*	//adis16488 单SPI时序
	uint16_t	cmd[1];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);

	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
*/
}

void
ADIS16488::modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits)
{
	uint16_t	val;

	val = read_reg16(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg16(reg, val);
}


int
ADIS16488::check_reg16(unsigned reg, uint16_t value)
{
	uint16_t val;

	val = read_reg16(reg);
	if(val == value)
	{
		return 1;
	}
	
	return 0;
}



int16_t
ADIS16488::convert12BitToINT16(int16_t word)
{

	int16_t outputbuffer = 0;

	if ((word >> 11) & 0x1) {
		outputbuffer = (word & 0xfff) | 0xf000;

	} else {
		outputbuffer = (word & 0x0fff);
	}

	return (outputbuffer);
}



void
ADIS16488::start()
{
	/* make sure we are stopped first */
	//uint32_t last_call_interval = _call_interval;
	stop();
	//_call_interval = last_call_interval;

	/* discard any stale data in the buffers */
	_gyro_reports->flush();
	_accel_reports->flush();
	_mag_reports->flush();

	/* start polling at the specified rate */
	
	hrt_call_every(&_accel_call, 1000, _accel_call_interval - ADIS16488_ACCEL_TIMER_REDUCTION, (hrt_callout)&ADIS16488::measure_trampoline, this);
	hrt_call_every(&_gyro_call, 1000, _gyro_call_interval - ADIS16488_GYRO_TIMER_REDUCTION, (hrt_callout)&ADIS16488::gyro_measure_trampoline, this);
	hrt_call_every(&_mag_call, 1000, _mag_call_interval, (hrt_callout)&ADIS16488::mag_measure_trampoline, this);

	//hrt_call_every(&_gyro_call, 3000, 4000, (hrt_callout)&ADIS16488::gyro_measure_trampoline, this);
	//hrt_call_every(&_mag_call, 1000, 3000, (hrt_callout)&ADIS16488::mag_measure_trampoline, this);

	warnx("hrt_call_every SUCCESS pass") ;

}

void
ADIS16488::stop()
{


	hrt_cancel(&_accel_call);
	hrt_cancel(&_gyro_call);
	hrt_cancel(&_mag_call);

	/* reset internal states */
	memset(_last_accel, 0, sizeof(_last_accel));
	memset(_last_gyro, 0, sizeof(_last_gyro));

	/* discard unread data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();
	_mag_reports->flush();

	
}

void
ADIS16488::measure_trampoline(void *arg)
{
	ADIS16488 *dev = reinterpret_cast<ADIS16488 *>(arg);
	//rain 2018-9-3 10:06:41
    //ADIS16448 *dev = (ADIS16448 *)(arg);
	//warnx("measure_trampoline");
	/* make another measurement */
	dev->measure();
}

void
ADIS16488::gyro_measure_trampoline(void *arg)
{
	ADIS16488 *dev = reinterpret_cast<ADIS16488 *>(arg);

	/* make another measurement */
	dev->gyro_measure();
}

void
ADIS16488::mag_measure_trampoline(void *arg)
{
	ADIS16488 *dev = reinterpret_cast<ADIS16488 *>(arg);

	/* make another measurement */
	dev->mag_measure();
}


int
ADIS16488::measure()
{

#pragma pack(push, 1)

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
	} report;
#pragma pack(pop)

	//rain 2018-9-6 14:29:09
	//adis16488_drdy_status = ADIS16488_DRDY;

	while(!ADIS16488_DRDY){
		//if adis16488_drdy_status == false
		//wait
		;
	}

	/* start measuring */
	perf_begin(_accel_sample_perf);

	memset(&report, 0, sizeof(report));


	/*
	 * Fetch the full set of measurements from the ADIS16448 in one pass (burst read).
	 */


	//	//rain 2018-9-4 16:33:44
	//获取adis16488高16位数据
	//	write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE0);
	report.accel_x = read_reg16(ADIS16488_XACCL_OUT);
	report.accel_y = read_reg16(ADIS16488_YACCL_OUT);
	report.accel_z = read_reg16(ADIS16488_ZACCL_OUT);


	//判断依据过于简单，需调整
	if (
	    report.accel_x == 0 && report.accel_y == 0 && report.accel_z == 0) {
		perf_count(_accel_bad_transfers);
		perf_end(_accel_sample_perf);
		return -EIO;
	}


	//rain 2018-8-30 14:15:49
	//打印adis16488寄存器中原始数据
	bool is_printf_adis16488 = false;
	if(is_printf_adis16488)
	{	
		PX4_INFO("report.accel_x: 0x%02x", report.accel_x);
		PX4_INFO("report.accel_y: 0x%02x", report.accel_y);
		PX4_INFO("report.accel_z: 0x%02x", report.accel_z);
	}

	
	//rain 2018-10-13
	//modify data limit
	/*
	 * Swap axes and negate y
	 */
	//int16_t accel_xt = report.accel_y;
	int16_t accel_yt = ((report.accel_y == -32768) ? 32767 : -report.accel_y);
	int16_t accel_zt = ((report.accel_z == -32768) ? 32767 : -report.accel_z);

	/*
	 * Apply the swap
	 */
	//report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.accel_z = accel_zt;



	 // *
	 // * Report buffers.
	 // * /
	accel_report	arb;

	arb.timestamp = hrt_absolute_time();
	arb.error_count = perf_event_count(_accel_bad_transfers);

	// * Accel report: * /
	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;


	float xraw_f = report.accel_x;	
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;



	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;


	_last_accel[0] = x_in_new;
	_last_accel[1] = y_in_new;
	_last_accel[2] = z_in_new;

	if (FW_FILTER) {
		arb.x = _accel_filter_x.apply(x_in_new);
		arb.y = _accel_filter_y.apply(y_in_new);
		arb.z = _accel_filter_z.apply(z_in_new);

	} else {
		arb.x = x_in_new;
		arb.y = y_in_new;
		arb.z = z_in_new;
	}

	arb.scaling = _accel_range_scale;
	//rain 2018-10-22 v1.8.0 delete .range_m_s2 variable
	//arb.range_m_s2 = _accel_range_m_s2;


	// * Temperature report: * /

	//rain 2018-10-22 v1.8.0 delete arb.temperature_raw
	//arb.temperature_raw = _last_temperature_raw;
	arb.temperature 	= _last_temperature;

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	// * return device ID * /
	arb.device_id = _device_id.devid;

	_accel_reports->force(&arb);


//	if (accel_notify) {
//		poll_notify(POLLIN);
//		;
//	}

	//rain 2018年9月10日10:19:07
	//此处的代码限制了acc速率的更新
	//poll_notify() just check other event 
	// * notify anyone waiting for data * /
	if (accel_notify) {
		poll_notify(POLLIN);
	}


	if (accel_notify && !(_pub_blocked)) {
		// * publish it * /
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);

		if(1==USER_TOPIC_LOG_ENABLE){
		orb_publish(ORB_ID(sensor_accel_adis16488), _accel_topic_adis16488, &arb);

		}
	}

	//rain2018-9-4 09:43:05
	//读取计数
	perf_count(_accel_reads);

	// * stop measuring * /
	perf_end(_accel_sample_perf);

	return OK;
}


int
ADIS16488::gyro_measure()
{

#pragma pack(push, 1)
	
	struct Report {
		int16_t 	gyro_x;
		int16_t 	gyro_y;
		int16_t 	gyro_z;
	} report;
#pragma pack(pop)


	//rain 2018-9-6 14:29:09
	//adis16488_drdy_status = ADIS16488_DRDY;

	while(!ADIS16488_DRDY){
		//if adis16488_drdy_status == false
		//wait
		//warnx("wait...");
		;
	}

	/* start measuring */
	perf_begin(_gyro_sample_perf);



	//rain 2018-9-4 16:16:32
	//读取adis16488输出数据寄存器数据-方式一
/**/
	//rain 2018-9-4 16:33:44
	//获取adis16488高16位数据
//	write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE0);
	report.gyro_x = read_reg16(ADIS16488_XGYRO_OUT);
	report.gyro_y = read_reg16(ADIS16488_YGYRO_OUT);
	report.gyro_z = read_reg16(ADIS16488_ZGYRO_OUT);


	if (report.gyro_x == 0 && report.gyro_y == 0 && report.gyro_z == 0 ) {
		perf_count(_gyro_bad_transfers);
		perf_end(_gyro_sample_perf);
		return -EIO;
	}


	//rain 2018-8-30 14:15:49
	//打印adis16488寄存器中原始数据
	bool is_printf_adis16488 = false;
	if(is_printf_adis16488)
	{
		printf("ADIS16448::measure is running");
		PX4_INFO("report.gyro_x: 0x%02x", report.gyro_x);
		PX4_INFO("report.gyro_y: 0x%02x", report.gyro_y);
		PX4_INFO("report.gyro_z: 0x%02x", report.gyro_z);

	}



	//rain 2018-10-13
	//modify data limit
	/*
	 * Swap axes and negate y
	 */
	//int16_t accel_xt = report.accel_y;
	int16_t gyro_yt = ((report.gyro_y == -32768) ? 32767 : -report.gyro_y);
	int16_t gyro_zt = ((report.gyro_z == -32768) ? 32767 : -report.gyro_z);

	/*
	 * Apply the swap
	 */
	//report.accel_x = accel_xt;
	report.gyro_y = gyro_yt;
	report.gyro_z = gyro_zt;



	//rain 2018-9-4 16:23:07
	//一次性读取acc ,gyro,mag数据
	 // *
	 // * Report buffers.
	 // * /
	gyro_report		grb;


	grb.timestamp = hrt_absolute_time();
	grb.error_count = perf_event_count(_gyro_bad_transfers);

	// * Gyro report: * /
	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	float xraw_f = report.gyro_x;
	float yraw_f = report.gyro_y;
	float zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale)  - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale)  - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale)  - _gyro_scale.z_offset) * _gyro_scale.z_scale;


	_last_gyro[0] = x_gyro_in_new;
	_last_gyro[1] = y_gyro_in_new;
	_last_gyro[2] = z_gyro_in_new;


	if (FW_FILTER) {
		grb.x = _gyro_filter_x.apply(x_gyro_in_new);
		grb.y = _gyro_filter_y.apply(y_gyro_in_new);
		grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	} else {
		grb.x = x_gyro_in_new;
		grb.y = y_gyro_in_new;
		grb.z = z_gyro_in_new;
	}

	grb.scaling = _gyro_range_scale ;
	
	//rain 2018-10-22 v1.8.0 delete grb.range_rad_s variable
	//grb.range_rad_s = _gyro_range_rad_s;
	

	// * Temperature report: * /
	//rain 2018-10-22 v1.8.0 delete grb.range_rad_s variable
	//grb.temperature_raw = _last_temperature_raw;
	grb.temperature 	= _last_temperature;


	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(grb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	// * return device ID * /
	grb.device_id = _gyro->_device_id.devid;

	_gyro_reports ->force(&grb);


	//rain 2018-9-5
	//add delay in order to confirm progress
	//up_udelay(500000);

	// * notify anyone waiting for data * /
	if (gyro_notify) {
		_gyro->parent_poll_notify();
	}


	if (gyro_notify && !(_pub_blocked)) {
		// * publish it * /
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
		if(1==USER_TOPIC_LOG_ENABLE){
			orb_publish(ORB_ID(sensor_gyro_adis16488), _gyro->_gyro_topic_adis16488, &grb);

		}		
		
		
	}


	//rain2018-9-4 09:43:05
	//读取计数
	perf_count(_gyro_reads);

	// * stop measuring * /
	perf_end(_gyro_sample_perf);

	return OK;
}


int
ADIS16488::mag_measure()
{
		
	struct Report {
		int16_t	    temp;
		int16_t 	mag_x;
		int16_t 	mag_y;
		int16_t 	mag_z;
	} report;

	//rain 2018-9-6 14:29:09
	//adis16488_drdy_status = ADIS16488_DRDY;

	while(!ADIS16488_DRDY){
		//if adis16488_drdy_status == false
		//wait
		//warnx("wait...");
		
	}

	/* start measuring */
	perf_begin(_mag_sample_perf);

	/*
	 * Fetch the full set of measurements from the ADIS16448 in one pass (burst read).
	 */
	//rain 2018-9-4 16:16:32
	//读取adis16488输出数据寄存器数据-方式一
	//rain 2018-9-4 16:33:44
	//获取adis16488高16位数据

	//write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE0);

	report.temp = read_reg16(ADIS16488_TEMP_OUT);

	report.mag_x = read_reg16(ADIS16488_XMAGN_OUT);
	report.mag_y = read_reg16(ADIS16488_YMAGN_OUT);
	report.mag_z = read_reg16(ADIS16488_ZMAGN_OUT);
	//report.baro    = (int16_t) adis_report.baro;





	if (report.mag_x == 0 && report.mag_y == 0 && report.mag_z == 0 && report.temp == 0) {
		perf_count(_mag_bad_transfers);
		perf_end(_mag_sample_perf);
		return -EIO;
	}


	//rain 2018-8-30 14:15:49
	//打印adis16488寄存器中原始数据
	bool is_printf_adis16488 = false;
	if(is_printf_adis16488)
	{
		PX4_INFO("report.mag_x: 0x%02x", report.mag_x);
		PX4_INFO("report.mag_y: 0x%02x", report.mag_y);
		PX4_INFO("report.mag_z: 0x%02x", report.mag_z);
		
		PX4_INFO("report.temp: 0x%02x", report.temp);

	}


	//rain 2018-10-13
	//modify data limit
	/*
	 * Swap axes and negate y
	 */
	//int16_t accel_xt = report.accel_y;
	int16_t mag_yt = ((report.mag_y == -32768) ? 32767 : -report.mag_y);
	int16_t mag_zt = ((report.mag_z == -32768) ? 32767 : -report.mag_z);

	/*
	 * Apply the swap
	 */
	//report.accel_x = accel_xt;
	report.mag_y = mag_yt;
	report.mag_z = mag_zt;
	//rain 2018-9-4 16:23:07

	 // *
	 // * Report buffers.
	 // * /

	mag_report		mrb;

	mrb.timestamp = hrt_absolute_time();
	mrb.error_count = perf_event_count(_mag_bad_transfers);

	// * Mag report: * /
	mrb.x_raw = report.mag_x;
	mrb.y_raw = report.mag_y;
	mrb.z_raw = report.mag_z;

	float xraw_f = report.mag_x;
	float yraw_f = report.mag_y;
	float zraw_f = report.mag_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	mrb.x = ((xraw_f * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mrb.y = ((yraw_f * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mrb.z = ((zraw_f * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;

	mrb.scaling  = _mag_range_scale;
	//rain 2018-10-22 v1.8.0 delete mrb.range_mgauss variable
	//mrb.range_ga = _mag_range_mgauss;
	//rain2018-9-8 15:15:47
	_last_temperature_raw = report.temp;
	_last_temperature = _last_temperature_raw * TEMPERATUREINITIALSENSITIVITY + 25;
	mrb.temperature 	= _last_temperature;
	

	// * return device ID * /
	mrb.device_id = _mag->_device_id.devid;

	_mag_reports  ->force(&mrb);

	// * notify anyone waiting for data * /

	_mag->parent_poll_notify();


	//rain 2018-9-6
	//modify mag topic publish 

	if (!(_pub_blocked)) {			// * Mag data validity bit (bit 8 DIAG_STAT) * /
		// * publish it * /
		orb_publish(ORB_ID(sensor_mag), _mag->_mag_topic, &mrb);
		
		if(1==USER_TOPIC_LOG_ENABLE){
			orb_publish(ORB_ID(sensor_mag_adis16488), _mag->_mag_topic_adis16488, &mrb);

		}		
		
	}


	//rain2018-9-4 09:43:05
	//读取计数
	perf_count(_mag_reads);


	// * stop measuring * /
	perf_end(_mag_sample_perf);

	return OK;
}






void
ADIS16488::print_calibration_data()
{
	uint16_t XGYRO_OFF = read_reg16(ADIS16448_XGYRO_OFF);
	uint16_t YGYRO_OFF = read_reg16(ADIS16448_YGYRO_OFF);
	uint16_t ZGYRO_OFF = read_reg16(ADIS16448_ZGYRO_OFF);
	uint16_t XACCL_OFF = read_reg16(ADIS16448_XACCL_OFF);
	uint16_t YACCL_OFF = read_reg16(ADIS16448_YACCL_OFF);
	uint16_t ZACCL_OFF = read_reg16(ADIS16448_ZACCL_OFF);
	uint16_t XMAGN_HIC = read_reg16(ADIS16448_XMAGN_HIC);
	uint16_t YMAGN_HIC = read_reg16(ADIS16448_YMAGN_HIC);
	uint16_t ZMAGN_HIC = read_reg16(ADIS16448_ZMAGN_HIC);
	uint16_t XMAGN_SIC = read_reg16(ADIS16448_XMAGN_SIC);
	uint16_t YMAGN_SIC = read_reg16(ADIS16448_YMAGN_SIC);
	uint16_t ZMAGN_SIC = read_reg16(ADIS16448_ZMAGN_SIC);

	warnx("single calibration value read:");
	warnx("XGYRO_OFF =:  \t%8.4x\t", XGYRO_OFF);
	warnx("YGYRO_OFF =:  \t%8.4x\t", YGYRO_OFF);
	warnx("ZGYRO_OFF =:  \t%8.4x\t", ZGYRO_OFF);
	warnx("XACCL_OFF =:  \t%8.4x\t", XACCL_OFF);
	warnx("YACCL_OFF =:  \t%8.4x\t", YACCL_OFF);
	warnx("ZACCL_OFF =:  \t%8.4x\t", ZACCL_OFF);
	warnx("XMAGN_HIC =:  \t%8.4x\t", XMAGN_HIC);
	warnx("YMAGN_HIC =:  \t%8.4x\t", YMAGN_HIC);
	warnx("ZMAGN_HIC =:  \t%8.4x\t", ZMAGN_HIC);
	warnx("XMAGN_SIC =:  \t%8.4x\t", XMAGN_SIC);
	warnx("YMAGN_SIC =:  \t%8.4x\t", YMAGN_SIC);
	warnx("ZMAGN_SIC =:  \t%8.4x\t", ZMAGN_SIC);
}

void
ADIS16488::print_info()
{
	perf_print_counter(_accel_sample_perf);
	perf_print_counter(_gyro_sample_perf);
	perf_print_counter(_mag_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_mag_reads);
	perf_print_counter(_accel_bad_transfers);
	perf_print_counter(_gyro_bad_transfers);
	perf_print_counter(_mag_bad_transfers);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	_mag_reports->print_info("mag queue");

	printf("DEVICE ID:\nACCEL:\t%d\nGYRO:\t%d\nMAG:\t%d\n", _device_id.devid, _gyro->_device_id.devid,
	       _mag->_device_id.devid);

	//rain 2018-10-19
   //读取filer_bank_register data,验证是否写入数据
   uint16_t i = 0, j = 0;
   
   uint16_t bank_a[120];
   memset(& bank_a, 0, sizeof(bank_a));
   
   write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE5);
   for(i=0; i<60; i++)
   {
	   bank_a[i] = read_reg16(i*2 + 0x08);
	   printf("bank_a[%d] = %d \t", i, bank_a[i]);
	   if(i % 4 == 0)
		   printf("\n");
   }
   
   write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE6);
   for(j=0; j<60; j++)
   {
	   bank_a[j+60] = read_reg16(j*2 + 0x08);
	   printf("bank_a[%d] = %d \t", j+60, bank_a[j]);
	   if(j % 4 == 0)
		   printf("\n");
   }
   write_reg16(ADIS16488_PAGE_ADDR, ADIS16488_PAGE0);



		   
}

ADIS16488_gyro::ADIS16488_gyro(ADIS16488 *parent, const char *path) :
	CDev("ADIS16488_gyro", path),
	_parent(parent),
	_gyro_topic(nullptr),
	_gyro_topic_adis16488(nullptr),
	_gyro_orb_class_instance(-1),
	_gyro_class_instance(-1)
{
		warnx("rain:ADIS16488_gyro::ADIS16488_gyro execute");
}

ADIS16488_gyro::~ADIS16488_gyro()
{
	if (_gyro_class_instance != -1) {
		unregister_class_devname(GYRO_BASE_DEVICE_PATH, _gyro_class_instance);
	}
}

int
ADIS16488_gyro::init()
{
	int ret;
	warnx("ADIS16448_gyro::init");
	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_gyro_class_instance = register_class_devname(GYRO_BASE_DEVICE_PATH);

	warnx("register_class_devname(GRYO_BASE_DEVICE_PATH) SUCCESS");
	return ret;
}

void
ADIS16488_gyro::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
ADIS16488_gyro::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->gyro_read(filp, buffer, buflen);
}

int
ADIS16488_gyro::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->gyro_ioctl(filp, cmd, arg);
	}
}

ADIS16488_mag::ADIS16488_mag(ADIS16488 *parent, const char *path) :
	CDev("ADIS16488_mag", path),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_topic_adis16488(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1)
{
		warnx("rain: ADIS16448_mag::ADIS16448_mag execute");
}

ADIS16488_mag::~ADIS16488_mag()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}
}

int
ADIS16488_mag::init()
{
	int ret;
	warnx("ADIS16448_mag::init");
	// do base class init
	ret = CDev::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	warnx("register_class_devname(MAG_BASE_DEVICE_PATH) SUCCESS");
	return ret;
}

void
ADIS16488_mag::parent_poll_notify()
{
	poll_notify(POLLIN);
}

ssize_t
ADIS16488_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	return _parent->mag_read(filp, buffer, buflen);
}


int
ADIS16488_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);
		break;

	default:
		return _parent->mag_ioctl(filp, cmd, arg);
	}
}

void
ADIS16488_mag::measure()
{
	_parent->mag_measure();
}

void
ADIS16488_mag::measure_trampoline(void *arg)
{
	_parent->mag_measure_trampoline(arg);
}


/**
 * Local functions in support of the shell command.
 */
namespace adis16488
{

ADIS16488	*g_dev;

void	start(enum Rotation rotation);
void	test();
void	reset();
void	info();
void 	info_cal();
void	usage();
/**
 * Start the driver.
 */
void
start(enum Rotation rotation)
{
	int fd, fd_mag,fd_gyro; 
	warnx("adis16488::start");
	if (g_dev != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
#if defined(PX4_SPI_BUS_EXT)
	g_dev = new ADIS16488(PX4_SPI_BUS_EXT, ADIS16488_DEVICE_PATH_ACCEL, ADIS16488_DEVICE_PATH_GYRO,
			      ADIS16488_DEVICE_PATH_MAG, PX4_SPIDEV_EXT_MPU, rotation);
#else
	PX4_ERR("External SPI not available");
	exit(0);
#endif

	if (g_dev == nullptr) {
		PX4_ERR("g_dev == nullptr");	
		goto fail;
	}

	if (OK != (g_dev)->init()) {
		PX4_ERR("OK != (g_dev)->init()");
		goto fail;
	}
	//up_udelay(500000);

	/* set the poll rate to default, starts automatic data collection */
	fd = open(ADIS16488_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("ADIS16488_DEVICE_PATH_ACCEL");
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0");
		goto fail;
	}

	//warnx("adis16488::start,open");
	//up_udelay(500000);

//rain 2018-9-2 17:01:34
//疑问：为什么有的传感器（lsm303d）打开了acc, mag文件,而有的只打开了acc的文件？

	// get the gyro driver 
	fd_gyro = open(ADIS16488_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd_gyro < 0) {
		PX4_ERR("ADIS16488_DEVICE_PATH_GYRO");
		goto fail;

	}
	if (ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("ioctl(fd_gyro, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0");
		goto fail;
	}


	/**/
	// get the mag driver
	fd_mag = open(ADIS16488_DEVICE_PATH_MAG, O_RDONLY);

	if (fd_mag < 0) {
		PX4_ERR("ADIS16488_DEVICE_PATH_MAG");
		goto fail;

	}	
	if (ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("ioctl(fd_mag, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0");
		goto fail;
	}

	
	/**/

	close(fd);

	//rain 2018-9-2
	close(fd_gyro);
	close(fd_mag);
	warnx("rain:void start exit");
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	accel_report a_report;
	gyro_report  g_report;
	mag_report 	 m_report;

	ssize_t sz;

	/* get the driver */
	int fd = open(ADIS16488_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed", ADIS16488_DEVICE_PATH_ACCEL);
	}

	/* get the mag driver */
	int fd_mag = open(ADIS16488_DEVICE_PATH_MAG, O_RDONLY);

	if (fd_mag < 0) {
		err(1, "%s open failed", ADIS16488_DEVICE_PATH_MAG);
	}

	/* get the gyro driver */
	int fd_gyro = open(ADIS16488_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", ADIS16488_DEVICE_PATH_GYRO);
	}

	/* do a simple demand read */
	sz = read(fd, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(a_report));
		err(1, "immediate acc read failed");
	}

	print_message(a_report);

	/* do a simple demand read */
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(g_report));
		err(1, "immediate gyro read failed");
	}

	print_message(g_report);

	/* do a simple mag demand read */
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		warnx("ret: %d, expected: %d", sz, sizeof(m_report));
		err(1, "immediate mag read failed");
	}

	print_message(m_report);

	/* XXX add poll-rate tests here too */
	close(fd_mag);
	close(fd_gyro);
	close(fd);

	reset();
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{

	warnx("namespace ADIS16448::reset() running");
	int fd = open(ADIS16488_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	usleep(10000);

	exit(0);
}

/**
 * Print a sensor calibration info.
 */
void
info_cal()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	g_dev->print_calibration_data();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'info_cal', 'reset',\n");
	warnx("options:");
	warnx("    -R rotation");
}

}
// namespace

int
adis16448_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int ch;

	/* start options */
	while ((ch = getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			adis16488::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		adis16488::start(rotation);
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		adis16488::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		adis16488::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		adis16488::info();
	}

	/*
	 * Print sensor calibration information.
	 */
	if (!strcmp(verb, "info_cal")) {
		adis16488::info_cal();
	}

	adis16488::usage();
 	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'info' or 'info_cal'");
}

