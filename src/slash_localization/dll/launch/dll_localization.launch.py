#!/usr/bin/env python3
"""
DLL (Direct Lidar Localization) Launch File
用于替代 AMCL 进行3D激光雷达定位

使用方法:
  ros2 launch dll dll_localization.launch.py map_path:=/path/to/your/map.bt
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    dll_share_dir = get_package_share_directory('dll')
    
    # 默认地图路径
    default_map_path = os.path.join(dll_share_dir, 'maps', 'test1.bt')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_path = LaunchConfiguration('map_path', default=default_map_path)
    
    # 点云输入话题
    in_cloud = LaunchConfiguration('in_cloud', default='/livox/lidar/pointcloud_filtered')
    
    # 坐标系配置
    base_frame_id = LaunchConfiguration('base_frame_id', default='base_link')
    odom_frame_id = LaunchConfiguration('odom_frame_id', default='odom')
    global_frame_id = LaunchConfiguration('global_frame_id', default='map')
    
    # 初始位姿
    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='0.0')
    initial_z = LaunchConfiguration('initial_z', default='0.0')
    initial_a = LaunchConfiguration('initial_a', default='0.0')  # yaw角
    
    # DLL节点
    dll_node = Node(
        package='dll',
        executable='dll_node',
        name='dll_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 地图配置
            'map_path': map_path,
            'global_frame_id': global_frame_id,
            
            # 点云输入
            'in_cloud': in_cloud,
            
            # 坐标系
            'base_frame_id': base_frame_id,
            'odom_frame_id': odom_frame_id,
            
            # 初始位姿
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_z': initial_z,
            'initial_a': initial_a,
            'initial_z_offset': 0.0,
            
            # IMU配置
            'use_imu': False,
            'use_yaw_increments': False,
            
            # 更新参数
            'update_rate': 20.0,
            'update_min_d': 0.1,  # 最小平移距离触发更新 (米)
            'update_min_a': 0.1,  # 最小旋转角度触发更新 (弧度)
            'update_min_time': 1.0,  # 最小时间间隔触发更新 (秒)
            
            # 配准方法: 1=DLL(需要预计算grid), 2=NDT(推荐), 3=ICP
            'align_method': 1,  # 使用NDT，无需预计算grid文件
            
            # 求解器参数
            'solver_max_iter': 75,
            'solver_max_threads': 8,
            
            # 可视化
            'publish_point_cloud': True,
            'publish_point_cloud_rate': 0.2,
            'publish_grid_slice': 0.3,  # 发布的栅格切片高度
            'publish_grid_slice_rate': 0.2,
        }],
        remappings=[
            ('initial_pose', '/initialpose'),  # 与RViz兼容
            ('imu/data', '/imu'),      # IMU数据话题
            ('odom', '/odometry/filtered'),        # 里程计话题
        ]
    )
    
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation clock if true'),
        DeclareLaunchArgument('map_path', default_value=default_map_path,
                             description='Full path to the octomap (.bt) file'),
        DeclareLaunchArgument('in_cloud', default_value='/livox/lidar/pointcloud_filtered',
                             description='Input point cloud topic'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link',
                             description='Base frame ID'),
        DeclareLaunchArgument('odom_frame_id', default_value='odom',
                             description='Odometry frame ID'),
        DeclareLaunchArgument('global_frame_id', default_value='map',
                             description='Global/map frame ID'),
        DeclareLaunchArgument('initial_x', default_value='0.0',
                             description='Initial X position'),
        DeclareLaunchArgument('initial_y', default_value='0.0',
                             description='Initial Y position'),
        DeclareLaunchArgument('initial_z', default_value='0.0',
                             description='Initial Z position'),
        DeclareLaunchArgument('initial_a', default_value='0.0',
                             description='Initial yaw angle (radians)'),
        
        # 启动节点
        dll_node,
    ])
