#!/bin/bash
echo "start simulation"

# echo "pub obstalce..."
# rostopic pub -1 /ldarc/obstacle std_msgs/Float32MultiArray "layout:
#   dim:
#   - label: ''
#     size: 0
#     stride: 0
#   data_offset: 0
# data: [0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.36]"


echo "take off ..."
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

sleep 4

echo "circle ..."
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0]"

# sleep 3
# echo "Slowdown ..."
# rostopic pub -1 /joy sensor_msgs/Joy "header:
#   seq: 0
#   stamp:
#     secs: 0
#     nsecs: 0
#   frame_id: \"/dev/input/js0\"
# axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
# buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]" 



sleep 6
echo "Slowdown ..."
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0 
  stamp:
    secs: 0
    nsecs: 0
  frame_id: \"/dev/input/js0\"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]" 



sleep 1

rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]"



sleep 1

rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0]"

