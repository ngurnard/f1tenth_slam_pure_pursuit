import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = get_package_share_directory('pure_pursuit')
    waypoint_dir = str(share_directory+"/../../../../src/lab-7-model-predictive-control-hot-wheels/pure_pursuit/waypoints/")
    # print("\n\nPATH:\n", waypoint_dir)
    config = os.path.join(
        share_directory,
        "../../../../",
        "src",
        "lab-7-model-predictive-control-hot-wheels",
        "pure_pursuit",
        "waypoints",
        "",
      )
    # print(config)

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
                'waypoints_path' : "/sim_ws/src/lab-7-model-predictive-control-hot-wheels/pure_pursuit/waypoints/",
                'waypoints_file' : "waypoints_mpc.csv",
                'v_csv'          :  1,
                }
            ]
        ),
        Node(
            package='pure_pursuit',
            executable='pose_fake_pub_node',
            name='pose_fake_pub_node',
        )
    ])