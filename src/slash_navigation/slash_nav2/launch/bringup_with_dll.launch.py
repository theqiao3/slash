#!/usr/bin/env python3
"""
使用DLL定位的Nav2导航启动文件
此文件用于替换AMCL，使用DLL进行3D激光雷达定位

使用方法:
  ros2 launch slash_nav2 bringup_with_dll.launch.py map_bt:=/path/to/octomap.bt
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    slash_nav2_dir = get_package_share_directory('slash_nav2')
    #nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    dll_dir = get_package_share_directory('dll')
    
    # 配置文件路径
    rviz_config_dir = os.path.join(slash_nav2_dir, 'rviz', 'nav2.rviz')
    nav2_param_path = os.path.join(slash_nav2_dir, 'config', 'nav2_params.yaml')
    
    # 默认地图路径
    default_map_yaml = os.path.join(slash_nav2_dir, 'map', 'test1.yaml')
    default_map_bt = os.path.join(dll_dir, 'maps', 'test1.bt')
    
    # Launch配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map_yaml', default=default_map_yaml)
    map_bt = LaunchConfiguration('map_bt', default=default_map_bt)
    params_file = LaunchConfiguration('params_file', default=nav2_param_path)
    
    # DLL参数
    in_cloud = LaunchConfiguration('in_cloud', default='/livox/lidar')
    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='0.0')
    initial_z = LaunchConfiguration('initial_z', default='0.0')
    initial_a = LaunchConfiguration('initial_a', default='0.0')
    
    # PCD可视化
    enable_pcd = LaunchConfiguration('enable_pcd', default='true')
    # 使用包内相对路径，避免硬编码工作区路径
    pcd_file_path = os.path.join(slash_nav2_dir, 'PCD', 'test1.pcd')
    
    # ==================== DLL定位节点 ====================
    dll_node = Node(
        package='dll',
        executable='dll_node',
        name='dll_node',
        output='log',  # 改为log，不在终端输出INFO信息
        arguments=['--ros-args', '--log-level', 'warn'],  # 只显示WARN及以上级别
        parameters=[{
            'use_sim_time': use_sim_time,
            # 地图配置
            'map_path': map_bt,
            'global_frame_id': 'map',
            
            # 点云输入
            'in_cloud': in_cloud,
            
            # 坐标系
            'base_frame_id': 'base_link',
            'odom_frame_id': 'lidar_odom',
            
            # 初始位姿
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_z': initial_z,
            'initial_a': initial_a,
            'initial_z_offset': 0.0,
            
            # IMU配置
            'use_imu': True,
            'use_yaw_increments': False,
            
            # 更新参数
            'update_rate': 10.0,
            'update_min_d': 0.1,
            'update_min_a': 0.1,
            'update_min_time': 1.0,
            
            # 配准方法: 1=DLL, 2=NDT, 3=ICP
            'align_method': 1,
            
            # 求解器参数
            'solver_max_iter': 75,
            'solver_max_threads': 8,
            
            # 可视化
            'publish_point_cloud': True,
            'publish_point_cloud_rate': 0.2,
            'publish_grid_slice': 0.3,
            'publish_grid_slice_rate': 0.2,
        }],
        remappings=[
            ('initial_pose', '/initialpose'),
        ]
    )
    
    # ==================== Nav2导航（不包含AMCL） ====================
    # 使用nav2_bringup但跳过localization部分
    # 我们需要单独启动各个Nav2组件
    
    # Map Server - 仍然需要2D地图用于全局代价地图
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }]
    )
    
    # Lifecycle Manager for map_server
    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    
    # Nav2 Navigation (不含定位)
    # nav2_navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'params_file': params_file,
    #     }.items()
    # )
    
    # ==================== RViz ====================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ==================== 速度校正节点 ====================
    vel_corrector_node = Node(
        package='slash_nav2',
        executable='vel_corrector.py',
        name='velocity_corrector_node',
        parameters=[
            {'min_hardware_speed': 0.8},
            {'correction_gain': 0.6}
        ],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('cmd_vel_corrective', '/cmd_vel_corrective')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        # ===== 参数声明 =====
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation clock if true'),
        DeclareLaunchArgument('map_yaml', default_value=default_map_yaml,
                             description='Full path to 2D map yaml file (for costmap)'),
        DeclareLaunchArgument('map_bt', default_value=default_map_bt,
                             description='Full path to 3D octomap (.bt) file (for DLL)'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                             description='Full path to Nav2 params file'),
        DeclareLaunchArgument('in_cloud', default_value='/livox/lidar',
                             description='Input point cloud topic for DLL'),
        DeclareLaunchArgument('initial_x', default_value='0.0',
                             description='Initial X position'),
        DeclareLaunchArgument('initial_y', default_value='0.0',
                             description='Initial Y position'),
        DeclareLaunchArgument('initial_z', default_value='0.0',
                             description='Initial Z position'),
        DeclareLaunchArgument('initial_a', default_value='0.0',
                             description='Initial yaw angle'),
        DeclareLaunchArgument('enable_pcd', default_value='true',
                             description='Enable PCD visualization'),
        
        # ===== 启动节点 =====
        # DLL定位
        dll_node,
        
        # Map Server
        map_server_node,
        lifecycle_manager_map,
        
        # Nav2导航
        #nav2_navigation_launch,
                # 使用本地修改后的bringup_launch.py（已屏蔽AMCL）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slash_nav2_dir, 'launch', 'bringup_launch.py')),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml,
                'use_sim_time': use_sim_time,
                'params_file': params_file}.items(),
        ),
        # PCD可视化
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(slash_nav2_dir, 'launch', 'bringup_real.launch.py')
            ]),
        ) if False else GroupAction([]),  # 暂时禁用，避免重复启动
        
        # 速度校正
        #vel_corrector_node,
        
        # RViz
        rviz_node,
    ])
