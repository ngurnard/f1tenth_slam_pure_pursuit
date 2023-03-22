import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    print("\n\nPATH:\n", os.path.join(
                    get_package_share_directory('pure_pursuit'),
                    'waypoints'))
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',  
        ),
        Node(
            package='pure_pursuit',
            executable='waypoint_node',
            name='waypoint_node',
            parameters=[
                {
                'source_frame'   : "map",
                'target_frame'   : "laser",
                'waypoints_path' : "/f1tenth_ws/src/pure_pursuit/pure_pursuit/waypoints/",
                'waypoints_file' : "waypoints_drive.csv"
                # 'waypoints_file' : DeclareLaunchArgument('number_of_uavs', default_value="waypoints_drive.csv")
                }
            ]
        ),
        IncludeLaunchDescription
        (
                PythonLaunchDescriptionSource
                ([
                    FindPackageShare("particle_filter"),
                    '/launch',
                    'localize_launch.py'
                ])
        ) 
    ])