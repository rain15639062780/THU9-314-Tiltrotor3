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


