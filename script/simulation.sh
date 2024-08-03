#!/bin/bash

# 获取当前脚本所在目录
SCRIPT_DIR="$(dirname "$(realpath "$0")")"

# 使用相对路径
source "$SCRIPT_DIR/../devel/setup.bash"

roslaunch cnuav_control simulation.launch 

