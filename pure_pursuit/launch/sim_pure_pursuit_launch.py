import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            parameters=[
            {
            'Kp' : 0.3,
            'v' : 1.0
            }
            ]
        ),
        Node(
            package='pure_pursuit',
            executable='waypoint_node',
            name='waypoint_node',
            parameters=[
                {
                'source_frame'   : "map",
                'target_frame'   : "ego_racecar/laser_model",
                'waypoints_path' : "/sim_ws/src/pure_pursuit/pure_pursuit/waypoints/",
                'waypoints_file' : "waypoints_gen.csv"
                }
            ]
        ),
        Node(
            package='pure_pursuit',
            executable='pose_fake_pub_node',
            name='pose_fake_pub_node',
        )
    ])