# catkin_ws

```
git clone https://github.com/alex34679/catkin_ws.gitmkdir catkin_ws
cd ./script
sudo ./init.sh
cd ../ && catkin_make
# 1.画圆
cd ./script && sudo ./simulation.sh
# 新开终端：
sudo ./circle.sh
# rviz中任意位置发送 2D Nav Goal

# 2.ego_planner
cd ./scirpt && sudo ./simulation.sh
# 新开终端
sudo ./ego.sh
# rviz中，发送2D Pose Estimate 添加障碍物
# 发送2D Nav Goal 发送目标点

```


