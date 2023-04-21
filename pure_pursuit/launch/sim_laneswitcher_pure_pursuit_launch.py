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
            }
            ]
        ),
        Node(
            package='pure_pursuit',
            executable='waypoint_laneswitcher_node',
            name='waypoint_laneswitcher_node',
            parameters=[
                {
                'global_frame'   : "map",
                'local_frame'    : "ego_racecar/laser_model",
                'waypoints_path' : share_directory,
                'v_csv'          :  1,
                'v'              :  1.0
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