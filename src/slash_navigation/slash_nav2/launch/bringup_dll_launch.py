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
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    slash_nav2_dir = get_package_share_directory('slash_nav2')
    launch_dir = os.path.join(slash_nav2_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    map_bt_file = LaunchConfiguration('map_bt')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # DLL parameters
    in_cloud = LaunchConfiguration('in_cloud')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    initial_a = LaunchConfiguration('initial_a')

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

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(slash_nav2_dir, 'map', 'test1.yaml'),
        description='Full path to 2D map yaml file to load')

    declare_map_bt_cmd = DeclareLaunchArgument(
        'map_bt',
        default_value=os.path.join(get_package_share_directory('dll'), 'maps', 'test1.bt'),
        description='Full path to 3D octomap (.bt) file for DLL')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(slash_nav2_dir, 'config', 'nav2_params_dll_teb.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

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

    declare_enable_pcd_cmd = DeclareLaunchArgument(
        'enable_pcd', default_value='true',
        description='Whether to enable PCD visualization')
    
    declare_pcd_file_cmd = DeclareLaunchArgument(
        'pcd_file', default_value=os.path.join(slash_nav2_dir, 'PCD', 'test1.pcd'),
        description='Full path to PCD file for visualization')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_dll_launch.py')),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'map_bt': map_bt_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_respawn': use_respawn,
                              'in_cloud': in_cloud,
                              'initial_x': initial_x,
                              'initial_y': initial_y,
                              'initial_z': initial_z,
                              'initial_a': initial_a,
                              'log_level': log_level}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        # 速度校正节点
        Node(
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
        ),

        # PCD点云发布节点
        Node(
            condition=IfCondition(LaunchConfiguration('enable_pcd')),
            package='slash_nav2',
            executable='publish_pcd.py',
            name='pcd_publisher',
            arguments=['--pcd-file', LaunchConfiguration('pcd_file'), '--topic', '/pcd_map', '--frame-id', 'map'],
            output='screen'
        ),
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            parameters=[{
                # octomap_server expects 'octomap_path' (not 'octomap_file')
                'octomap_path': os.path.join(get_package_share_directory('dll'), 'maps', 'test1.bt'),
                'frame_id': 'map',
                # 确保地图在启动时被发布并保留（便于后续订阅）
                'latch': True,
                # 可选参数，按需启用/调整
                'publish_free_space': False,
                'resolution': 0.05,
            }],
            output='screen'
        ),
    ])

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_map_bt_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_in_cloud_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_initial_z_cmd)
    ld.add_action(declare_initial_a_cmd)
    ld.add_action(declare_enable_pcd_cmd)
    ld.add_action(declare_pcd_file_cmd)

    ld.add_action(bringup_cmd_group)
    return ld
