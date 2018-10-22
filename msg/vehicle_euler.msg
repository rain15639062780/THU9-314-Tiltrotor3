# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use

uint64 timestamp		# time since system start (microseconds)
float32 roll	# Bias corrected angular about X body axis in rad
float32 pitch	# Bias corrected angular about Y body axis in rad
float32 yaw	# Bias corrected angular about Z body axis in rad
float32[4] q		# Quaternion rotation from NED earth frame to XYZ body frame
# TOPICS vehicle_euler

