# Copyright 2023
# Author: MA Song

import os

from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('frl_rosbot_onboard')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params_pro.yaml')
    map_yaml_file = os.path.join(pkg_dir, 'config', 'map', 'mpeb417_map.yaml')

    rosbot_description = get_package_share_directory('rosbot_description')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/rosbot_pro.launch.py']),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'base_footprint'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup, '/launch/bringup_launch.py']),
            launch_arguments={
                'params_file': params_file,
                'map': map_yaml_file
            }.items()
        )
    ])
