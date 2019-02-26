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

3.2018-12-23 测试pwm_capture启动后是否发布pwm_capture消息及是否开始记录数据。

修改文件：src/modules/logger/logger.cpp
中添加了pwm_capture消息的头文件及add_topic()语句

实验结果：保存的日志数据中并未包含pwm_capture消息。
推测：pwm_capture_main函数中只是简单启动cap口功能，然后通过printf函数打印在nsh终端,并未对数据进行publish.

rain 2019-1-30
4.1	源程序修改说明（Github:THU9-314-Tiltrotor3/THU9-314-tiltrotor-imu-cap-v1.0）
源文件位置	修改简述
Src/drivers/pwm_capture/	                Pwm捕捉功能硬件驱动程序
Src/drivers/propeller_test/	                订阅adc,pwm输出,control,pwm脉宽等消息，初始化串口（baud =57600）,数据处理，发送数据帧（仅发送功能）
ROMFS/px4fmu_common/mixers/quad_+.main.mix	将ch5的控制源更改为油门，调整偏移量和上下限
ROMFS/px4fmu_common/Init.d/5001_quad_+	    设置第5 pwm输出通道频率为400Hz
Cmake/configs/nuttx_px4fmu-v5_default.cmake	将添加的程序加入自动编译脚本
ROMFS/px4fmu_common/Init.d/rcS	            将功能函数启动命令加入自启动脚本

4.2	数据通信协议：

字节序号	  定义	         比例尺	    范围
1	        帧头55H		
2	        帧头 AAH		
0~1	        控制量-roll	    1000	-1.0~1.0
2~3	        控制量-pitch	1000	-1.0~1.0
4~5	        控制量-yaw	    1000	-1.0~1.0
6~7	        控制量-thrust	1000	0.0~1.0
			
8~9	        S1通道输出pwm	1	    0~2000
10~11	    S2通道输出pwm	1	    0~2000
12~13	    S3通道输出pwm	1	    0~2000
14~15	    S4通道输出pwm	1	    0~2000
			
16~19	    捕获脉宽周期值	 u64	
			
20~21	    电压ad采集值	1	    0~4096
			
22~23	    电流ad采集值	1	    0~4096
24	        校验和	u8	

4.3	硬件连接
    飞控电源板---传感器
    ADC3v3---电压信号
	ADC6v6---电流信号
	Cap1---转速反馈信号
	M5&GND---电调控制信号
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
螺旋桨测试装置油门控制函数添加记录

在mc_att_control_main.cpp中添加油门梯度控制函数，主要包括
	/*rain 2019-2-25
	*程序中判断aux1状态，切入不同处理程序
	*处理程序中修改control[3]的值
	*注意：在飞行器实飞过程中必须关闭此部分
	*/

1.在.h函数中定义变量，构造函数中初始化，run()函数中实现功能函数，更新control[3]
2.地面站使用aux旋钮控制，当aux<-0.5时，control[3]递减，当aux>0.5时，control[3]递增


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



