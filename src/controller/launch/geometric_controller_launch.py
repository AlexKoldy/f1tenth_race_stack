from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """"""
    launch_description = LaunchDescription(
        [
            Node(
                package="controller",
                executable="geometric_controller_node",
                name="geometric_controller",
                output="screen",
                parameters=[
                    {"alpha": 0.25},  # [s]
                    {"beta": -0.1},  # [m]
                    {"ell_min": 1.0},  # [m]
                    {"ell_max": 5.0},  # [m]
                    {"k_p": 1.0},  # [m]
                    {"controller_name": "pure_pursuit"},
                ],
            ),
        ]
    )
    return launch_description
