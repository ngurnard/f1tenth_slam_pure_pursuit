import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    share_directory = os.path.join(
        get_package_share_directory('pure_pursuit'),
        'waypoints', "")
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            parameters=[
            {
            'Kp' : 0.4
            }
            ]
        ),
        Node(
            package='pure_pursuit',
            executable='waypoint_node',
            name='waypoint_node',
            parameters=[
                {
                'global_frame'   : "map",
                'local_frame'    : "laser",
                'waypoints_path' : share_directory,
                'waypoints_file' : "trial_raceline.csv",
                'v_csv'          : 1,
                'v'              : 3.0,
                'L'              : 1.2,
                'L_csv'          : 1
                }
            ]
        ),
        # Node(
        #     package='pure_pursuit',
        #     executable='bagger.py',
        #     name='bagger',
        # ),
        IncludeLaunchDescription
        (
                PythonLaunchDescriptionSource
                ([
                    FindPackageShare("particle_filter"),
                    '/launch',
                    '/localize_launch.py'
                ])
        ) 
    ])
