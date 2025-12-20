import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 配置文件路径
    config_dir = os.path.join(get_package_share_directory('slash_nav2'), 'config')
    processing_config = os.path.join(config_dir, 'grid_map_processing.yaml')
    visualization_config = os.path.join(config_dir, 'grid_map_visualization.yaml')
    
    # PCL 配置文件路径 (使用 grid_map_demos 的默认配置)
    grid_map_demos_dir = get_package_share_directory('grid_map_demos')
    pcl_config = os.path.join(grid_map_demos_dir, 'config', 'realtime_pcl_grid_config.yaml')

    return LaunchDescription([
        # Grid Map 处理节点 (从点云生成 Grid Map 并计算坡度)
        Node(
            package='grid_map_demos',
            executable='pointcloud2_to_gridmap_demo',
            name='grid_map_processor',
            parameters=[processing_config,
                        {
                        "config_file_path": pcl_config
                        }
                    ],
            output='screen'
        ),
        
        # Grid Map 可视化节点
        Node(
            package='grid_map_visualization',
            executable='grid_map_visualization',
            name='grid_map_visualizer',
            parameters=[visualization_config],
            output='screen'
        )
    ])
