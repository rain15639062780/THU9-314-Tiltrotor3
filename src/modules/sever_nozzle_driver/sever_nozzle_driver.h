#include <stdint.h>
#include <string.h>



/*****************************************
 * struct : Servo_data
 * 作者：THU rain
 * 描述：用于记录舵机数据。与舵机发送相关数据相一致。
 * 日期：2018-9-15 14:12:24
 ***************************************** */
struct Servo_data {
	uint8_t head1;
	uint8_t head2;
	uint8_t pos_l;
	uint8_t pos_h;
	uint8_t i_main_l;
	uint8_t i_main_h;
	uint8_t num;
	uint8_t checksum;
};
