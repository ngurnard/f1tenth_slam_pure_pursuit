import os
import rospkg

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


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
            'Kp' : 0.2,
            'drive_topic' : "/drive",
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
                'local_frame'    : "ego_racecar/laser_model",
                'waypoints_path' : share_directory,
                'waypoints_file' : "waypoints_raceline_2.csv",
                'v_csv'          :  0,
                'v'              :  3.0
                }
            ],
            output='screen',
        ),
        Node(
            package='pure_pursuit',
            executable='waypoint_node',
            name='waypoint_node',
            parameters=[
                {
                'global_frame'   : "map",
                'local_frame'    : "opp_racecar/laser_model",
                'waypoints_path' : share_directory,
                'waypoints_file' : "waypoints_raceline_2.csv",
                'v_csv'          :  0,
                'v'              :  4.0
                }
            ],
            output='screen',
        ),
        Node(
            package='pure_pursuit',
            executable='pose_fake_pub_node',
            name='pose_fake_pub_node',
        )
    ])