#!/bin/bash
echo "start simulation"

#起飞
echo "take off ..."
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

sleep 8

#正常画圆
echo "circle ..."
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]"

sleep 5

echo "Foce hover"
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]"

sleep 1

echo "change"
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]"

sleep 2

#在新的配置下画圆
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]"

sleep 4

#先悬停
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]"
sleep 1

#变回原有阵形
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]"

sleep 3

#新配置下画圆
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]"

sleep 4

#降落
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]"
sleep 2
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]"