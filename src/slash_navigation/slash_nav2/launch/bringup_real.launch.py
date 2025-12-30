import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # 获取与拼接默认路径
    slash_nav2_dir = get_package_share_directory(
        'slash_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz_config_dir = os.path.join(
        slash_nav2_dir, 'rviz', 'nav2.rviz')
    
    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='false')
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(slash_nav2_dir, 'map', 'test2.yaml'))
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(slash_nav2_dir, 'config', 'nav2_params (teb).yaml'))
    
    # PCD点云地图参数 - 使用包内相对路径（更便于移植）
    pcd_file_path = os.path.join(slash_nav2_dir, 'PCD', 'test1.pcd')
    pcd_file = launch.substitutions.LaunchConfiguration(
        'pcd_file', default=pcd_file_path)
    enable_pcd = launch.substitutions.LaunchConfiguration(
        'enable_pcd', default='true')  # 默认启用PCD发布

    # twist_node = Node(
    #     package='twist_to_ackermann',
    #     executable='twist_to_ackermann_node',
    #     name='twist_to_ackermann',
    #     parameters=[
    #         {'cmd_vel_topic': '/tianracer/cmd_vel'},
    #         {'drive_topic': 'drive'},
    #         {'use_sim_time': use_sim_time}
    #     ]
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

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

    return launch.LaunchDescription([
        # 声明新的 Launch 参数
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value='false',
                                             description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument('map', default_value=map_yaml_path,
                                             description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument('params_file', default_value=nav2_param_path,
                                             description='Full path to param file to load'),
        launch.actions.DeclareLaunchArgument('enable_pcd', default_value='true',
                                             description='Enable PCD point cloud visualization (default: true)'),
        launch.actions.DeclareLaunchArgument('pcd_file', default_value=pcd_file_path,
                                             description='Path to PCD file for visualization'),

        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        # PCD点云发布节点（使用 ros2 run 命令）
        # 使用 Node 直接运行已安装的脚本，避免硬编码工作区路径
        Node(
            condition=launch.conditions.IfCondition(enable_pcd),
            package='slash_nav2',
            executable='publish_pcd.py',
            name='pcd_publisher',
            arguments=['--pcd-file', pcd_file, '--topic', '/pcd_map', '--frame-id', 'map'],
            output='screen'
        ),
        # twist_node,
        vel_corrector_node,
        rviz_node,
        # Node(
        #     package='nav2_waypoint_follower',
        #     executable='waypoint_follower',
        #     output='screen'
        #     )
    ])
