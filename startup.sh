#!/bin/bash
# 一键启动雷达驱动 + 雷达管线
# 使用：./startup.sh

SESSION="radar"

tmux new-session -d -s $SESSION -n "driver" "ros2 launch livox_ros2_driver livox_lidar_launch.py"
tmux new-window -t $SESSION -n "lidar" "ros2 launch dynamic_cloud lidar.launch.py"

tmux attach -t $SESSION
