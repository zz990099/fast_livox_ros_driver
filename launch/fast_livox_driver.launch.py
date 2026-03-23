# Copyright 2026 zz990099
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('fast_livox_ros_driver')

    namespace = LaunchConfiguration('namespace')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')
    livox_config_file = LaunchConfiguration('livox_config_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='True',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            package_dir, 'config', 'fast_livox_driver_params.yaml'
        ),
        description='Full path to the parameter file for fast_livox_driver',
    )

    declare_livox_config_file_cmd = DeclareLaunchArgument(
        'livox_config_file',
        default_value=os.path.join(package_dir, 'config', 'MID360_config.json'),
        description='Full path to the config file for mid360 lidar',
    )

    load_fast_livox_driver_node = (
        Node(
            namespace=namespace,
            package='fast_livox_ros_driver',
            executable='fast_livox_ros_driver_node',
            name='fast_livox_ros_driver',
            output='screen',
            parameters=[
                params_file,
                {'config_file_path': livox_config_file},
            ],
            respawn=use_respawn,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', log_level],
        ),
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_livox_config_file_cmd)

    ld.add_action(load_fast_livox_driver_node)

    return ld
