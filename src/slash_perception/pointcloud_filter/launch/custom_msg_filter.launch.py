#!/usr/bin/env python3
"""
Launch文件：启动Livox CustomMsg点云过滤节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_filter',
            executable='custom_msg_filter_node',
            name='custom_msg_filter_node',
            output='screen',
            parameters=[{
                'min_radius': 0.2,  # 最小半径（米）
                'input_topic': '/livox/lidar',
                'output_topic': '/livox/lidar_filtered',
            }]
        ),
    ])
