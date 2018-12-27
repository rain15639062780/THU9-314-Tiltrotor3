# This topic records the angular acceleration estimation signals.
# Created by rain, 2018-12-27

uint64 timestamp		# time since system start (microseconds)

#内环角速率rates
float32[3] body_rates     #roll, pitch, yaw  body axis 

#内环滤低通波角速率rates_lp
float32[3] body_rates_lp     #roll, pitch, yaw  body axis
 
#角速率融合模块 模型预测角加速度
float32[3] ang_acc_dq_model  #roll, pitch, yaw  body axis
 
#角速率融合模块 模型预测角速度
float32[3] ang_acc_q  #roll, pitch, yaw  body axis

#角速率融合模块 模型预测 最内环 预测角加速度
float32[3] ang_acc_dq_estimate  #roll, pitch, yaw  body axis

#角速率融合模块 模型预测 次内环 预测角加速度
float32[3] ang_acc_dq_estimat  #roll, pitch, yaw  body axis

#角速率融合模块 模型预测 次内环 预测角加速度输出
float32[3] ang_acc_d_u  #roll, pitch, yaw  body axis
