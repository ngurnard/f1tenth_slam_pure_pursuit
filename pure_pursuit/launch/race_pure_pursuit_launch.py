from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            remappings=[
            ("pf/viz/inferred_pose", "pose")
            ]
        ),
        Node(
            package='pure_pursuit',
            executable='waypoint_node',
            name='waypoint_node',
            remappings=[
            ("pf/viz/inferred_pose", "pose")
            ]
        )
    ])