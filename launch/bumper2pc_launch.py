from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bumper2pc",
            executable="bumper2pc",
            name="bumper2pc",
            output="screen",
            emulate_tty=True,
            parameters=[
                { "pointcloud_radius": 0.3,
                  "pointcloud_height": 0.05,
                  "side_point_angle": 0.35,
                  "base_link_frame": "base_link"
                }
            ]
        )
    ])