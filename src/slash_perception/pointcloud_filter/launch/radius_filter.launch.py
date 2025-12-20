#!/usr/bin/env python3
"""
Launch文件：启动PointCloud2点云过滤节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_filter',
            executable='radius_filter_node',
            name='radius_filter_node',
            output='screen',
            parameters=[{
                'min_radius': 0.2,  # 最小半径（米）
                'input_topic': '/livox/lidar/pointcloud',
                'output_topic': '/livox/lidar/pointcloud_filtered',
            }]
        ),
    ])
