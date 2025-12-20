import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('icp_localization')
    
    # Arguments
    pcd_map_path_arg = DeclareLaunchArgument(
        'pcd_map_path',
        default_value=os.path.join(get_package_share_directory('slash_nav2'), 'PCD', 'test1.pcd'),
        description='Path to the PCD map file'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'icp_localization.yaml'),
        description='Path to the config file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_share, 'rviz', 'icp_localization.rviz'),
        description='Path to the RViz config file'
    )

    # Nodes
    icp_node = Node(
        package='icp_localization',
        executable='icp_localization_node',
        name='icp_localization',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'pcd_map_path': LaunchConfiguration('pcd_map_path')}
        ]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        pcd_map_path_arg,
        config_file_arg,
        rviz_config_arg,
        icp_node,
        rviz_node
    ])
