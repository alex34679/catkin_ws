#!/bin/bash

# 获取当前脚本所在目录
SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# 使用相对路径
source "$SCRIPT_DIR/../devel/setup.bash"


echo "take off ..."
rostopic pub -1 /joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: "/dev/input/js0"
axes: [0.0, 0.0, 0.0, 0.0, -0.0, 0.0, 0.0, 0.0]
buttons: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]"


roslaunch ego_planner simple_run.launch 
