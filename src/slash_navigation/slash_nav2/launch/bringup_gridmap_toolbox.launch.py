#!/usr/bin/env python3
"""
使用 SLAM Toolbox 定位的 Nav2 导航启动文件
此文件用于替换 AMCL，使用 SLAM Toolbox 的定位模式进行激光定位

TF链: map -> odom -> base_link
     (SLAM Toolbox)  (车辆里程计/FAST_LIO)

使用方法:
  ros2 launch slash_nav2 bringup_with_slam_toolbox.launch.py
  
  # 指定序列化地图文件（需要先用 SLAM Toolbox 建图并保存）
  ros2 launch slash_nav2 bringup_with_slam_toolbox.launch.py map_posegraph:=/path/to/your_map

注意: 
  - map_posegraph 参数不需要带扩展名，会自动加载 .posegraph 和 .data 文件
  - 如果没有 .posegraph 文件，需要先使用 SLAM Toolbox 建图模式生成
  - 2D yaml 地图 (test1.yaml) 用于全局代价地图的 static_layer
  - .posegraph 地图用于 SLAM Toolbox 定位
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # 获取包路径
    slash_nav2_dir = get_package_share_directory('slash_nav2')
    
    # 配置文件路径
    rviz_config_dir = os.path.join(slash_nav2_dir, 'rviz', 'nav2.rviz')
    nav2_param_path = os.path.join(slash_nav2_dir, 'config', 'nav2_params_gridmap.yaml')
    
    # Grid Map 配置文件路径
    grid_map_processing_config = os.path.join(slash_nav2_dir, 'config', 'grid_map_processing.yaml')
    grid_map_visualization_config = os.path.join(slash_nav2_dir, 'config', 'grid_map_visualization.yaml')
    
    # PCL 配置文件路径 (使用 grid_map_demos 的默认配置)
    grid_map_demos_dir = get_package_share_directory('grid_map_demos')
    pcl_config = os.path.join(grid_map_demos_dir, 'config', 'realtime_pcl_grid_config.yaml')
    
    # 默认地图路径
    default_map_yaml = os.path.join(slash_nav2_dir, 'map', 'test1.yaml')
    # 注意：如果没有 .posegraph 文件，需要先用 SLAM Toolbox 建图
    # 可以使用 RMUC.posegraph 作为示例（如果存在）
    default_map_posegraph = os.path.join(slash_nav2_dir, 'map', 'test1')  # 不带扩展名
    
    # PCD点云地图参数 - 使用包内相对路径（更便于移植）
    pcd_file_path = os.path.join(slash_nav2_dir, 'PCD', 'test1.pcd')
    # Launch substitutions (PCD file and enabling flag)
    pcd_file = LaunchConfiguration('pcd_file', default=pcd_file_path)
    enable_pcd = LaunchConfiguration('enable_pcd', default='true')  # 默认启用PCD发布
    
    # Launch配置
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map_yaml', default=default_map_yaml)
    map_posegraph = LaunchConfiguration('map_posegraph', default=default_map_posegraph)
    params_file = LaunchConfiguration('params_file', default=nav2_param_path)

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
    
    
    return LaunchDescription([
        # ===== 参数声明 =====
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation clock if true'),
        DeclareLaunchArgument('map_yaml', default_value=default_map_yaml,
                             description='Full path to 2D map yaml file (for global costmap static_layer)'),
        DeclareLaunchArgument('map_posegraph', default_value=default_map_posegraph,
                             description='Full path to SLAM Toolbox posegraph file (without extension, for localization)'),
        DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                             description='Full path to Nav2 params file'),
        DeclareLaunchArgument('enable_pcd', default_value='true',
                             description='Enable PCD point cloud visualization'),
        DeclareLaunchArgument('enable_rviz', default_value='true',
                             description='Enable RViz visualization'),
        
        # ===== 调用 bringup_launch.py =====
        # bringup_launch.py 会调用:
        #   - localization_launch.py (启动 map_server + slam_toolbox + lifecycle_manager)
        #   - navigation_launch.py (启动 controller_server, planner_server 等)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slash_nav2_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_yaml,                    # 2D yaml 地图给 map_server (全局代价地图)
                'map_posegraph': map_posegraph,     # posegraph 地图给 SLAM Toolbox (定位)
                'use_sim_time': use_sim_time,
                'params_file': params_file,
            }.items(),
        ),
        
        # ===== 速度校正 =====
        # vel_corrector_node,  # 按需启用
        
        # ===== PCD 可视化 =====
                # PCD点云发布节点（使用 ros2 run 命令）
        Node(
            condition=IfCondition(enable_pcd),
            package='slash_nav2',
            executable='publish_pcd.py',
            name='pcd_publisher',
            arguments=['--pcd-file', pcd_file, '--topic', '/pcd_map', '--frame-id', 'map'],
            output='screen'
        ),
        
        # ===== Grid Map =====
        grid_map_processor_node,
        grid_map_visualizer_node,

        # ===== RViz =====
        rviz_node,
    ])
