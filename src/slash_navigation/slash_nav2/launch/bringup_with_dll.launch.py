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
    nav2_param_path = os.path.join(slash_nav2_dir, 'config', 'nav2_params_dll_teb.yaml')
    
    # Grid Map 配置文件路径
    grid_map_processing_config = os.path.join(slash_nav2_dir, 'config', 'grid_map_processing.yaml')
    pcd_grid_map_processing_config = os.path.join(slash_nav2_dir, 'config', 'pcd_grid_map_processing.yaml')
    grid_map_visualization_config = os.path.join(slash_nav2_dir, 'config', 'grid_map_visualization.yaml')
    
    # PCL 配置文件路径 (使用 grid_map_demos 的默认配置)
    grid_map_demos_dir = get_package_share_directory('grid_map_demos')
    pcl_config = os.path.join(grid_map_demos_dir, 'config', 'realtime_pcl_grid_config.yaml')
    
    # 默认地图路径
    default_map_yaml = os.path.join(slash_nav2_dir, 'map', 'test2.yaml')
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
    pcd_file = LaunchConfiguration('pcd_file', default=pcd_file_path)

    # 可视化开关
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    
    # ==================== RViz ====================
    rviz_node = Node(
        condition=IfCondition(enable_rviz),
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

    # ==================== Grid Map Nodes ====================
    # Grid Map 处理节点 (从点云生成 Grid Map 并计算坡度)
    grid_map_processor_node = Node(
        package='grid_map_demos',
        executable='pointcloud2_to_gridmap_demo',
        name='grid_map_processor',
        parameters=[grid_map_processing_config,
                    {
                    "config_file_path": pcl_config
                    }
                ],
        output='screen'
    )
    
    # Grid Map 可视化节点
    grid_map_visualizer_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualizer',
        parameters=[grid_map_visualization_config],
        output='screen'
    )

    # PCD Grid Map 处理节点 (从 PCD 加载并生成全局 Grid Map)
    pcd_grid_map_processor_node = Node(
        package='grid_map_demos',
        executable='pcd_to_gridmap_demo',
        name='pcd_grid_map_processor',
        parameters=[pcd_grid_map_processing_config,
                    {
                    "config_file_path": pcl_config,
                    "pcd_file_path": pcd_file_path,
                    "map_frame_id": "map"
                    }
                ],
        remappings=[
            ("grid_map", "/global_grid_map")
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
        DeclareLaunchArgument('pcd_file', default_value=pcd_file_path,
                             description='Path to PCD file for visualization'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
                             description='Enable RViz visualization'),
        
        # ===== 启动核心导航与定位 =====
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slash_nav2_dir, 'launch', 'bringup_dll_launch.py')),
            launch_arguments={
                'map': map_yaml,
                'map_bt': map_bt,
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'in_cloud': in_cloud,
                'initial_x': initial_x,
                'initial_y': initial_y,
                'initial_z': initial_z,
                'initial_a': initial_a,
                'enable_pcd': enable_pcd,
                'pcd_file': pcd_file,
            }.items(),
        ),
        
        # ===== Grid Map =====
        grid_map_processor_node,
        grid_map_visualizer_node,
        pcd_grid_map_processor_node,
        
        # ===== RViz =====
        rviz_node,
    ])
