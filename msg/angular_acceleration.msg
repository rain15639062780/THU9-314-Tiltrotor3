# This topic records the angular acceleration estimation signals.
# Created by Yunjie Yang, 2018-09-15

uint64 timestamp		# time since system start (microseconds)

# Motor thrust mapped from pwm
float32[4] motor_thrust

# Angular acceleration from differential of angular velocity, rad/s^2
float32 roll_acc_diff     #roll, X body axis 
float32 pitch_acc_diff    #pitch, Y body axis
float32 yaw_acc_diff      #yaw, Z body axis

# Angular acceleration after low pass filter of differential value, with three different frequencies.
float32[3] roll_acc_lp     #roll, X body axis 
float32[3] pitch_acc_lp    #pitch, Y body axis
float32[3] yaw_acc_lp      #yaw, Z body axis

# Angular acceleration from prediction of motor thrust, rad/s^2
float32 roll_acc_predict     #roll, X body axis 
float32 pitch_acc_predict    #pitch, Y body axis
float32 yaw_acc_predict      #yaw, Z body axis

# Angular acceleration after high pass filter of predicted value, with three different frequencies.
float32[3] roll_acc_hp     #roll, X body axis 
float32[3] pitch_acc_hp    #pitch, Y body axis
float32[3] yaw_acc_hp      #yaw, Z body axis

# Angular acceleration after complementary filter, with three different frequencies.
float32[3] roll_acc_cf     #roll, X body axis 
float32[3] pitch_acc_cf    #pitch, Y body axis
float32[3] yaw_acc_cf      #yaw, Z body axis
