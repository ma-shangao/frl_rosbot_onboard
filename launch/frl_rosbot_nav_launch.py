import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch


def generate_launch_description():
    frl_rosbot_onboard = get_package_share_directory('frl_rosbot_onboard')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    slam_toolbox = get_package_share_directory('slam_toolbox')

    
    frl_bringup = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(frl_rosbot_onboard, 'launch', 'frl_rosbot_bringup_launch.py'))
    )

    nav2 = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py'))
    )

    slam = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox, 'launch', 'online_async_launch.py'))
    )


    return LaunchDescription([
        frl_bringup,
        nav2,
        slam,

    ])

if __name__ == '__main__':
    generate_launch_description()
