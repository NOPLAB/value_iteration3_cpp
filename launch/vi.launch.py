import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("value_iteration3"), "config", "params.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="value_iteration3",
                executable="vi_node",
                name="vi_node",
                parameters=[config],
                output="screen",
            )
        ]
    )
