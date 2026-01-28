#!/usr/bin/env python

"""
Launch file for offboard_control_boat node with parameters from YAML file.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 直接使用源码目录的绝对路径，确保始终读取最新的yaml文件
    config_file = '/home/xld/ws_off/src/px4_ros_com/config/offboard_control_boat.yaml'
    
    # 创建节点
    offboard_control_boat_node = Node(
        package='px4_ros_com',
        executable='offboard_control_boat',
        name='offboard_control_boat',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        offboard_control_boat_node
    ])
