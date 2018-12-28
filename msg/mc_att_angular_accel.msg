# This topic records the angular acceleration estimation signals.
# Created by rain, 2018-12-27

uint64 timestamp		# time since system start (microseconds)

#内环期望角速率rates
float32[3] body_rates_sp     #roll, pitch, yaw  body axis 

#内环角速率rates
float32[3] body_rates     #roll, pitch, yaw  body axis 

#内环滤低通波角速率rates_lp
float32[3] body_rates_lp     #roll, pitch, yaw  body axis
 
#角速率融合模块输入参数1： 模型预测 角加速度 
float32[3] ang_acc_dq_model  #roll, pitch, yaw  body axis
 
#角速率融合模块输入参数1： 模型预测角速度
float32[3] ang_acc_q_fbk  #roll, pitch, yaw  body axis

#角速率融合模块 模型预测 内环 滤波器输入
float32[3] ang_acc_err_u  #roll, pitch, yaw  body axis
float32[3] ang_acc_err_u_prev  #roll, pitch, yaw  body axis


#角速率融合模块 模型预测 内环 滤波器输出
float32[3] ang_acc_err_y  #roll, pitch, yaw  body axis
float32[3] ang_acc_err_y_prev  #roll, pitch, yaw  body axis

#角速率融合模块 模型预测 内环 预测角加速度输出
float32[3] ang_acc_d_est  #roll, pitch, yaw  body axis
float32[3] ang_acc_d_est_prev   #roll, pitch, yaw  body axis

#角速率融合模块 模型预测 内环 积分器
float32[3] ang_acc_int  #roll, pitch, yaw  body axis
float32[3] ang_acc_int_prev  #roll, pitch, yaw  body axis

#角速率融合模块 模型预测 次内环 角速度
float32[3] ang_acc_dq_estemt  #roll, pitch, yaw  body axis

#角速率融合模块 模型预测 次内环 角速度补偿
float32[3] ang_acc_d_u  #roll, pitch, yaw  body axis
