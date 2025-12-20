#!/usr/bin/env python3
"""
Launch file for SLAM Toolbox mapping with perception stack:
 - fast_lio/mapping.launch.py                          : FAST_LIO 里程计
 - linefit_ground_segmentation_ros/segmentation.launch.py : 地面分割
 - pointcloud_to_laserscan/pointcloud_to_laserscan_launch.py : 点云转激光扫描
 - slam_toolbox/online_async_launch.py                 : SLAM Toolbox 建图
 - rviz2                                               : 可视化

功能说明:
  - FAST_LIO 提供 lidar_odom -> base_link 的激光里程计
  - 地面分割过滤掉地面点，提供障碍物信息
  - 点云转激光扫描将 3D 点云转换为 2D 激光扫描供 SLAM Toolbox 使用
  - SLAM Toolbox 在建图模式下进行 SLAM，生成位姿图和地图
  
建图完成后，使用以下命令保存地图（序列化格式）:
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/tianbot/slash_ws/src/slash_navigation/slash_nav2/map/your_map_name'}"

使用方法:
  ros2 launch slash_nav2 bringup_all_mapping.launch.py
  
  # 可选参数：
  # - enable_rviz: 是否启用 RViz 可视化 (default: true)
  # - map_name: 保存地图时的基础名称 (default: test_map)
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    
    # ==================== 参数声明 ====================
    enable_rviz = LaunchConfiguration('enable_rviz', default='true')
    map_name = LaunchConfiguration('map_name', default='test1')
    
    # 获取包路径
    slash_nav2_dir = get_package_share_directory('slash_nav2')
    rviz_config_dir = os.path.join(slash_nav2_dir, 'rviz', 'nav2.rviz')
    slam_toolbox_config = os.path.join(slash_nav2_dir, 'config', 'mapping.yaml')  # 使用建图配置
    
    # ==================== 感知栈启动文件 ====================
    perception_launches = [
        ("fast_lio", "mapping.launch.py"),
        ("linefit_ground_segmentation_ros", "segmentation.launch.py"),
        ("pointcloud_to_laserscan", "pointcloud_to_laserscan_launch.py"),
    ]
    
    for pkg, lf in perception_launches:
        try:
            pkg_share = get_package_share_directory(pkg)
        except Exception as e:
            ld.add_action(LogInfo(msg=f"Package '{pkg}' not found: {e}"))
            continue

        launch_path = os.path.join(pkg_share, 'launch', lf)

        if not os.path.exists(launch_path):
            ld.add_action(LogInfo(msg=f"Launch file not found: {launch_path}"))
            continue

        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path)
        ))
    
    # ==================== SLAM Toolbox 建图模式 ====================
    # 注意：SLAM Toolbox 需要从 slam_toolbox 包的官方启动文件启动
    # 官方建图启动文件通常为 online_async_launch.py 或 online_launch.py
    try:
        slam_toolbox_dir = get_package_share_directory('slam_toolbox')
        slam_toolbox_launch = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        
        if os.path.exists(slam_toolbox_launch):
            # 使用官方的建图启动文件
            ld.add_action(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                launch_arguments={
                    'slam_params_file': slam_toolbox_config,
                    'use_sim_time': 'false',
                }.items()
            ))
        else:
            # 如果官方文件不存在，尝试使用备用文件
            slam_toolbox_launch_alt = os.path.join(slam_toolbox_dir, 'launch', 'online_launch.py')
            if os.path.exists(slam_toolbox_launch_alt):
                ld.add_action(IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(slam_toolbox_launch_alt),
                    launch_arguments={
                        'slam_params_file': slam_toolbox_config,
                        'use_sim_time': 'false',
                    }.items()
                ))
            else:
                ld.add_action(LogInfo(msg="SLAM Toolbox launch file not found"))
    except Exception as e:
        ld.add_action(LogInfo(msg=f"SLAM Toolbox package not found: {e}"))
    
    # ==================== RViz 可视化 ====================
    rviz_node = Node(
        condition=IfCondition(enable_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}],  # bool 类型，不是 string
        output='screen'
    )
    
    # ==================== 建图提示信息 ====================
    mapping_info_node = Node(
        package='ros2_common_interfaces',
        executable='echo',
        name='mapping_info',
        output='screen',
        arguments=[
            "echo '========================================';",
            "echo 'SLAM Toolbox Mapping Started!';",
            "echo '========================================';",
            "echo '';",
            "echo 'TF Chain: map -> lidar_odom -> base_link';",
            "echo '';",
            "echo 'After mapping is complete, save the map with:';",
            "echo '';",
            "echo 'ros2 service call /slam_toolbox/serialize_map \\';",
            "echo '  slam_toolbox/srv/SerializePoseGraph \\';",
            "echo '  \"{filename: \\'/home/tianbot/slash_ws/src/slash_navigation/slash_nav2/map/YOUR_MAP_NAME\\'}\"';",
            "echo '';",
            "echo 'Then use bringup_with_slam_toolbox.launch.py for localization';",
            "echo '========================================';",
        ]
    )
    
    # ==================== 声明参数 ====================
    ld.add_action(DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'map_name',
        default_value='test_map',
        description='Map name for saving (without extension)'
    ))
    
    # ==================== 添加启动节点 ====================
    ld.add_action(rviz_node)
    
    return ld
