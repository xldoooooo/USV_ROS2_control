#!/usr/bin/env python

"""
Launch file for offboard_control_boat node with parameters from YAML file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 直接使用源码目录的绝对路径，确保始终读取最新的yaml文件
    config_file = '/home/xld/ws_offboard/src/USV_ROS2_control/px4_ros_com/config/offboard_control_boat.yaml'
    
    # 使用 ExecuteProcess 在当前终端启动节点，支持键盘输入
    offboard_control_boat_process = ExecuteProcess(
        cmd=['bash', '-i', '-c', 
             'ros2 run px4_ros_com offboard_control_boat --ros-args --params-file ' + config_file],
        output='screen',
        shell=False,
        emulate_tty=True
    )
    
    return LaunchDescription([
        offboard_control_boat_process
    ])
