#!/bin/bash
echo "start test"

# echo "pub obstalce..."
# rostopic pub -1 /ldarc/obstacle std_msgs/Float32MultiArray "layout:
#   dim:
#   - label: ''
#     size: 0
#     stride: 0
#   data_offset: 0
# data: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.36]"

# 飞行器1： 1m起飞，并完成圆轨迹
echo "take off ..."
rostopic pub -1 /fpv_bh/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

sleep 2

# 飞行器2： 原点处起飞，保持原点悬停
rostopic pub -1 /warrior/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

sleep 10

echo "circle ..."
rostopic pub -1 /fpv_bh/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]"

sleep 20

echo "land ..."
rostopic pub -1 /fpv_bh/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]"

sleep 2


# 都降落
rostopic pub -1 /fpv_bh/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

rostopic pub -1 /warrior/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

# echo "go to [1.5, 0, 0.5]"
# rostopic pub -1 /ldarc/point geometry_msgs/Point "x: 1.5
# y: 0
# z: 0.5"


# sleep 2

# echo "go to [-1.5, 0.0, 0.5]"
# rostopic pub -1 /ldarc/point geometry_msgs/Point "x: -1.5
# y: 0.0
# z: 0.5"
