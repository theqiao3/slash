# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    slash_nav2_dir = get_package_share_directory('slash_nav2')
    dll_dir = get_package_share_directory('dll')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    map_bt_file = LaunchConfiguration('map_bt')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # DLL parameters
    in_cloud = LaunchConfiguration('in_cloud')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    initial_a = LaunchConfiguration('initial_a')

    lifecycle_nodes = ['map_server']

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(slash_nav2_dir, 'map', 'test1.yaml'),
        description='Full path to 2D map yaml file to load')

    declare_map_bt_cmd = DeclareLaunchArgument(
        'map_bt',
        default_value=os.path.join(dll_dir, 'maps', 'test1.bt'),
        description='Full path to 3D octomap (.bt) file for DLL')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(slash_nav2_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_in_cloud_cmd = DeclareLaunchArgument(
        'in_cloud', default_value='/livox/lidar',
        description='Input point cloud topic for DLL')

    declare_initial_x_cmd = DeclareLaunchArgument('initial_x', default_value='0.0')
    declare_initial_y_cmd = DeclareLaunchArgument('initial_y', default_value='0.0')
    declare_initial_z_cmd = DeclareLaunchArgument('initial_z', default_value='0.0')
    declare_initial_a_cmd = DeclareLaunchArgument('initial_a', default_value='0.0')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            
            Node(
                package='dll',
                executable='dll_node',
                name='dll_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'map_path': map_bt_file,
                    'global_frame_id': 'map',
                    'in_cloud': in_cloud,
                    'base_frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'global_frame_id': 'map',
                    'initial_x': initial_x,
                    'initial_y': initial_y,
                    'initial_z': initial_z,
                    'initial_a': initial_a,
                    'initial_z_offset': 0.0,
                    'use_imu': False,
                    'use_yaw_increments': False,
                    'update_rate': 10.0,
                    'update_min_d': 0.1,
                    'update_min_a': 0.1,
                    'update_min_time': 1.0,
                    'align_method': 1,
                    'solver_max_iter': 75,
                    'solver_max_threads': 8,
                    'publish_point_cloud': True,
                    'publish_point_cloud_rate': 5.0,
                    'publish_grid_slice': 0.3,
                    'publish_grid_slice_rate': 5.0,
                }],
                remappings=[
                    ('initial_pose', '/initialpose'),
                ],
                arguments=['--ros-args', '--log-level', log_level]
            ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}])
        ]
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_map_bt_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_in_cloud_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_initial_z_cmd)
    ld.add_action(declare_initial_a_cmd)

    ld.add_action(load_nodes)

    # 强制激活生命周期节点 (针对 lifecycle_manager 可能失效的情况)
    force_activate_nodes = GroupAction([
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
            output='screen'),
    ])
    
    # 延迟 5 秒执行，确保节点已经完全启动
    ld.add_action(TimerAction(period=5.0, actions=[force_activate_nodes]))

    return ld
