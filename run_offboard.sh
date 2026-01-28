#!/bin/bash

# 离线控制船只 - 快速启动脚本
# 用法: ./run_offboard.sh

WORKSPACE_DIR="/home/xld/ws_offboard"
CONFIG_FILE="$WORKSPACE_DIR/src/USV_ROS2_control/px4_ros_com/config/offboard_control_boat.yaml"

# 进入工作目录
cd "$WORKSPACE_DIR"

# 加载 ROS2 环境
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"

echo ""
echo "=========================================="
echo "启动离线控制船只节点"
echo "=========================================="
echo ""
echo "操作说明："
echo "  1. 按空格键（不需要按enter）"
echo "  2. 输入目标坐标 x y（例如: 5.0 3.0）"
echo "  3. 按回车确认"
echo ""
echo "按 Ctrl+C 退出"
echo ""

# 启动节点
ros2 run px4_ros_com offboard_control_boat --ros-args --params-file "$CONFIG_FILE"
