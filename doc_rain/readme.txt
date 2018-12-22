1.@ulog2csv_sh.sh
#ulog日志文件转换为csv数据文件

2.
#rain 2018-11-13 
@/home/rain-3/THU9-314/src1.8.0/THU9-314-Tiltrotor3/src/modules/mc_att_control
为了匹配飞控测试平台测试单通道的应用，在line772对即将发布出的vehicle_controls topic中屏蔽其他通道的值。



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
1. 2018-12-21 新创建仓库：THU9-314-Tiltrotor-imu_cap-v1.0,用来测试pixhawk v4的pwm捕捉功能，
用于后期螺旋桨测试平台转速测试。
操作：git pull origin THU9-314-Tiltrotor-imu-beta1


2. 2018-12-22 添加pwm-capture的相关文件
新增文件：
（1）src/drivers/pwm_capture/pwm_capture.cpp   pwm_capture初始化函数，修改pwm_input.cpp而得
（2）src/drivers/pwm_capture/CMakeList.txt     修改nsh终端显示的commader，设置栈空间大小
（3）src/drivers/drv_pwm_capture_rain.h		设置pwm_capture字符设备的地址
（4）msg/pwm_capture.msg			定义pwm_capture数据的结构形式
修改文件：
（1）msg/CMakeList.txt                          修改/添加pwm_capture.msg的编译
（2）cmake/congig/nuttx_px4fmu_v5_default.camke  修改/添加pwm_capture.cpp的编译
测试结果：
通过系统编译，板载CAP1端口可以正确获取输入脉宽数据信息
