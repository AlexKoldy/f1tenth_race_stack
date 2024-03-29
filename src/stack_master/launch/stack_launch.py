from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """"""
    # TODO: make this a command-line argument
    mode = "sim"

    # --------------------Geometric Controller Parameters-------------------#
    geometric_controller_params = [
        {"alpha": 0.25},  # lookahead distance slope [s]
        {"beta": -0.1},  # lookahead distance bias [m]
        {"ell_min": 1.0},  # minimum lookahead distance [m]
        {"ell_max": 5.0},  # minimum lookahead distance [m]
        {"k_p": 1.0},  # steering gain [m]
        {"controller_name": "pure_pursuit"},  # type of controller to use
        {"mode": mode},  # whether we are running in simulation or on the physical car
    ]

    # --------------------Global Planner Parameters-------------------#
    global_planner_params = [
        {"alpha_min": -0.25},  # minimum deviation from unoptimized path
        {"alpha_max": 0.25},  # maximum deviation from unoptimized path
        {"num_waypoints": 100},  # number of waypoints to use for optimized path
        {
            "v_x_min": 2
        },  # minimum longitudinal velocity for velocity profile generation [m/s]
        {
            "v_x_max": 5
        },  # maximum longitudinal velocity for velocity profile generation [m/s]
        {
            "a_x_max": 3
        },  # maximum longitudinal acceleration for velocity profile generation [m/s^2]
        {
            "a_y_max": 3
        },  # maximum lateral acceleration for velocity profile generation [m/s^2]
        {
            "optimized_trajectory_path": None
        },  # path to the optimized trajectory for loading, if None, we reoptimize
    ]

    if mode == "SIM":
        print("yes")
    elif mode == "REAL":
        print("real")

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
