# Copyright 2023 UCL Field Robotics and Learning Group
# Author: BAI Xianjian and MA Song

import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# from launch.actions import IncludeLaunchDescription
# from launch.conditions import IfCondition
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node


def generate_launch_description():
    # use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    rosbot_description = get_package_share_directory('rosbot_description')
    rplidar_ros = get_package_share_directory('rplidar_ros')
    frl_rosbot_onboard = get_package_share_directory('frl_rosbot_onboard')

    rosserial = launch_ros.actions.Node(
        package='rosbot_description',
        executable='rosserial_node.py',
        output='screen',
        parameters=[
            rosbot_description + '/config/rosserial_pro.yaml'
            ]
    )

    astra_camera = launch_ros.actions.Node(
        package='astra_camera',
        executable='astra_camera_node',
        output='screen'
    )

    rosbot_tf = launch_ros.actions.Node(
        package='rosbot_description',
        executable='rosbot_tf',
        output='log',
    )

    rp_lidar = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(rplidar_ros, 'launch', 'rplidar_a3.launch.py'))
    )

    rosbot_state = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            [frl_rosbot_onboard, '/launch/robot_state_publisher_launch.py'])
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        astra_camera,
        rosbot_state,
        rp_lidar,
        rosserial,
        rosbot_tf,

    ])
