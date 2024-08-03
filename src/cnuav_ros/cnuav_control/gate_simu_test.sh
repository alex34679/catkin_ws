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

sleep 6
#准备穿门
echo "ready to cross gate ..."
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]"

sleep 3
#穿门
echo "cross"
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]"

sleep 3
#改变门的位置
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'gate0'
  pose:
    position:
      x: 0.0
      y: 2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7071
      w: 0.7071
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'world'"

rosservice call /gazebo/set_model_state "model_state:
  model_name: 'gate0'
  pose:
    position:
      x: 0.0
      y: 2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.7071
      w: 0.7071
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'world'"
sleep 1
#准备穿门
echo "ready to cross gate ..."
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0]"


sleep 3
#穿门
echo "cross"
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]"

