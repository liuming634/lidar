#!/bin/bash
set -e

# ============================================
# rosbag 测试脚本
# 先编译，再用 rosbag 回放测试全部节点
# ============================================

# 修复 libusb 冲突：确保系统 libusb 优先于 MVS 自带的
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

echo "===== 编译 ====="
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1

echo "===== source 环境 ====="
source install/setup.bash

echo "===== 启动节点 ====="
# 相机检测（从 rosbag 读取图像）
ros2 launch tdt_vision run_rosbag.launch.py &
PID_CAM=$!

# LiDAR 识别管线（定位 + 动态点 + 聚类）
ros2 launch dynamic_cloud lidar.launch.py &
PID_LIDAR=$!

# 地图可视化
ros2 run debug_map debug_map &
PID_MAP=$!

echo "===== 全部已启动 ====="
echo "  run_rosbag  PID: $PID_CAM"
echo "  lidar       PID: $PID_LIDAR"
echo "  debug_map   PID: $PID_MAP"
echo ""
echo "按 Ctrl+C 停止所有节点"

# 等待任意一个进程退出，然后清理
trap "echo '正在停止...'; kill $PID_CAM $PID_LIDAR $PID_MAP 2>/dev/null; wait" EXIT
wait
