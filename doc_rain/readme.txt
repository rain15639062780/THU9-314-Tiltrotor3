1.@ulog2csv_sh.sh
#ulog日志文件转换为csv数据文件

2.
#rain 2018-11-13 
@/home/rain-3/THU9-314/src1.8.0/THU9-314-Tiltrotor3/src/modules/mc_att_control
为了匹配飞控测试平台测试单通道的应用，在line772对即将发布出的vehicle_controls topic中屏蔽其他通道的值。

//////////////////////////////////////////////////////////////
节点1：2018-12-25 14：51 提交

Branch：THU9-314-tiltrotor-imu-beta1
主要内容：该分支主要用于研究imu adis16488的替换工作，继承与master分支，目前该分支已经包括imu的驱动程序，频率提高的方案及备份文件，
室内测试平台的mix文件。能成功编译并下载v2及v5的固件。
注意：pixhawk-v4解锁开关与之前不太一样，建议使用最新版qgc，设置两档开关解锁。

//////////////////////////////////////////////////////////////
Rain 2018-12-27 角加速度数据融合模块程序相关修改备忘录

1.	新建mc_att_angular_accel.msg文件，
定义需要记录的数据类型


2.	修改mc_att_control.hpp文件
添加消息头文件，定义结构体变量，定义publish标识，补充成员函数。


			
		Vector3f ang_acc_dq_model;
		Vector3f ang_acc_q;
		Vector3f ang_acc_q1;
		Vector3f ang_acc_q1_prev;
		Vector3f ang_acc_q2;
		Vector3f ang_acc_q2_prev;
		Vector3f ang_acc_dq_estimate;
				Vector3f ang_acc_dq_estimate_prev;

//消息发布使用的变量 
	orb_advert_t	_v_angular_accel_pub{nullptr};		/**< angular_accel publication */

struct mc_att_angular_accel_s	_v_angular_accel {};		/**< vehicle attitude angular_accel */

	//定义入口变量类型 

	matrix::Vector3f ang_acc_dq_model;
	matrix::Vector3f ang_acc_q;
	matrix::Vector3f ang_acc_q1;
	matrix::Vector3f ang_acc_q1_prev;
	matrix::Vector3f ang_acc_q2;
	matrix::Vector3f ang_acc_q2_prev;
	matrix::Vector3f ang_acc_dq_estimate;
		matrix::Vector3f ang_acc_dq_estimate_prev;


3.	mc_att_angular_accel.cpp
（1）初始化mc_att_control.h中定义的三维矩阵变量，部分变量已在.h文件中初始化
（2）编写数据融合模块程序，最后需添加到mc_att_control.cpp中

注意：由于要使用到MulticopterAttitudeControl::control_attitude_rates(float dt)
中的局部变量，所以暂时把
MulticopterAttitudeControl::angular_accel_calculate(float dt)；
MulticopterAttitudeControl::angular_accel_publish(float dt)；
成员函数中的程序合并后放在
MulticopterAttitudeControl::control_attitude_rates(float dt)
中。待模块程序调试完成后在考虑程序模块间解耦合。



4.	修改logger.cpp中 msg消息的记录
（1）	添加头文件
（2）	开启记录add_topic（” ”）;默认设定最大频率


初步测试已经验证可以正常记录mc_att_angular_accel.msg中的数据，记录频率约为230Hz
//////////////////////////////////////////////////////////////////////////


