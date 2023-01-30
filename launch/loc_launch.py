from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
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
            PythonLaunchDescriptionSource([nav2_bringup, '/launch/localization_launch.py']),
            launch_arguments={
                'params_file': './src/frl_rosbot_onboard/config/amcl.yaml',
                'map': './map/mpeb417_map.yaml'
            }.items()  
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_bringup, '/launch/navigation_launch.py']),
        #     launch_arguments={
        #         'map_subscribe_transient_local': 'true'
        #     }.items()  
        # )
    ])
