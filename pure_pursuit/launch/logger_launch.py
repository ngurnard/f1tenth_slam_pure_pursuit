import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # share_directory = os.path.join(
    #     get_package_share_directory('pure_pursuit'),
    #     'waypoints', "")
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='waypoint_logger.py',
            name='waypoint_logger',
        ),
        Node(
            package='pure_pursuit',
            executable='lidar_logger.py',
            name='lidar_logger',
        )
        # IncludeLaunchDescription
        # (
        #         PythonLaunchDescriptionSource
        #         ([
        #             FindPackageShare("particle_filter"),
        #             '/launch',
        #             '/localize_launch.py'
        #         ])
        # ) 
    ])
