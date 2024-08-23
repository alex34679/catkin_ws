# 实际运行的数据
geometry_msgs/Vector3[] actual_position
geometry_msgs/Vector3[] actual_velocity
geometry_msgs/Vector3[] actual_acceleration
geometry_msgs/Quaternion[] actual_orientation

# 参考轨迹的数据
geometry_msgs/Vector3[] reference_position
geometry_msgs/Vector3[] reference_velocity
geometry_msgs/Vector3[] reference_acceleration
geometry_msgs/Quaternion[] reference_orientation

std_msgs/Float64[] actual_time


# 时间戳（可选）
std_msgs/Header header
