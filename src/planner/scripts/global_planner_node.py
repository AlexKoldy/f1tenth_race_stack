import rclpy
from rclpy.node import Node
from rclpy import Parameter
import numpy as np
import os
from typing import Dict, Union

# TODO: fix this so that it points to the proper place
# from planner.planner.global_planners.trajectory_builder import TrajectoryBuilder

from trajectory_builder import TrajectoryBuilder

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class GlobalPlannerNode(Node):
    def __init__(self) -> None:
        """"""
        super().__init__("global_planner_node")

        # Declare trajectory builder parameters
        self.declare_parameter("alpha_min", -0.15)
        self.declare_parameter("alpha_max", 0.15)
        self.declare_parameter("num_waypoints", 1500)
        self.declare_parameter("v_x_min", 2.0)
        self.declare_parameter("v_x_max", 5.0)
        self.declare_parameter("a_x_accel_max", 3.0)
        self.declare_parameter("a_x_decel_max", 3.0)
        self.declare_parameter("a_y_max", 3.0)
        self.declare_parameter("num_iterations", 1)
        self.declare_parameter("trajectory_load_file", "raw_waypoints_1.npz")
        self.declare_parameter("trajectory_save_file", "optimized_trajectory.npz")
        self.declare_parameter("reoptimize", True)

        # Set up parameters
        self.parmameters = self.set_params()

        # Set up topics
        self.path_topic = "/planner/global/path"
        self.velocity_profile_topic = "planner/global/velocity_profile"

        # Set up publishers, subscribers, timers
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.velocity_profile_pub = self.create_publisher(
            Path, self.velocity_profile_topic, 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

        # If we have a valid trajectory file, we can load it
        current_directory = os.path.dirname(os.path.abspath(__file__))
        self.trajectory_directory = os.path.join(
            current_directory, "..", "trajectories"
        )
        trajectory_load_file = os.path.join(
            self.trajectory_directory, self.trajectory_load_file
        )
        try:
            trajectory_data = np.load(trajectory_load_file)
            self.path = trajectory_data["path"]
            self.velocity_profile = trajectory_data["velocity_profile"]
        except:
            raise Exception(
                f"Either invalid or unfound trajectory file: {trajectory_load_file}"
            )

        # Perform initial trajectory optimization if specified
        if self.reoptimize:
            self.generate_trajectory()

    def set_params(self) -> Dict[str, Union[float, int, str, bool]]:
        """"""
        self.alpha_min = (
            self.get_parameter("alpha_min").get_parameter_value().double_value
        )
        self.alpha_max = (
            self.get_parameter("alpha_max").get_parameter_value().double_value
        )
        self.num_waypoints = (
            self.get_parameter("num_waypoints").get_parameter_value().integer_value
        )
        self.v_x_min = self.get_parameter("v_x_min").get_parameter_value().double_value
        self.v_x_max = self.get_parameter("v_x_max").get_parameter_value().double_value
        self.a_x_accel_max = (
            self.get_parameter("a_x_accel_max").get_parameter_value().double_value
        )
        self.a_x_decel_max = (
            self.get_parameter("a_x_decel_max").get_parameter_value().double_value
        )
        self.a_y_max = self.get_parameter("a_y_max").get_parameter_value().double_value
        self.num_iterations = (
            self.get_parameter("num_iterations").get_parameter_value().integer_value
        )
        self.trajectory_load_file = (
            self.get_parameter("trajectory_load_file")
            .get_parameter_value()
            .string_value
        )
        self.trajectory_save_file = (
            self.get_parameter("trajectory_save_file")
            .get_parameter_value()
            .string_value
        )
        self.reoptimize = (
            self.get_parameter("reoptimize").get_parameter_value().bool_value
        )
        return {
            "alpha_min": self.alpha_min,
            "alpha_max": self.alpha_max,
            "num_waypoints": self.num_waypoints,
            "v_x_min": self.v_x_min,
            "v_x_max": self.v_x_max,
            "a_x_accel_max": self.a_x_accel_max,
            "a_x_decel_max": self.a_x_decel_max,
            "a_y_max": self.a_y_max,
            "num_iterations": self.num_iterations,
        }

    def generate_trajectory(self) -> None:
        """"""
        # Set up the trajectory builder
        self.trajectory_builder = TrajectoryBuilder(
            self.alpha_min,
            self.alpha_max,
            self.a_x_accel_max,
            self.a_x_decel_max,
            self.a_y_max,
            self.v_x_min,
            self.v_x_max,
        )

        # Optimize our trajectory
        self.path, self.velocity_profile = self.trajectory_builder.generate_trajectory(
            self.path, self.num_waypoints, self.num_iterations
        )

        # Save our trajectory
        trajectory_save_file = os.path.join(
            self.trajectory_directory, self.trajectory_save_file
        )
        np.savez(
            trajectory_save_file, path=self.path, velocity_profile=self.velocity_profile
        )
        # TODO: visualize velocity profile with RVIZ somehow (different colored points??)
        # self.trajectory_builder.plot_paths()
        # self.trajectory_builder.plot_optimized_trajectory()
        # exit()

        # Turn off reoptimize to prevent further reoptimization
        self.set_parameters([Parameter("reoptimize", Parameter.Type.BOOL, False)])
        self.reoptimize = False

    def timer_callback(self) -> None:
        """"""
        # Set our parameters
        new_parameters = self.set_params()

        # If we have changed any of the parameters, reoptimize if that option is 'True'
        if self.reoptimize:
            self.generate_trajectory()
            # if not self.parmameters == new_parameters:
            #     self.generate_trajectory()

        # If we have a trajectory, publish it
        self.publish_trajectory(self.path, self.velocity_profile)

    def publish_trajectory(
        self, path: np.ndarray, velocity_profile: np.ndarray
    ) -> None:
        """"""
        # Set up the path and velocity profile message
        path_msg = Path()
        path_msg.header.stamp = rclpy.time.Time().to_msg()
        path_msg.header.frame_id = "map"

        velocity_profile_msg = Path()
        velocity_profile_msg.header.stamp = path_msg.header.stamp
        velocity_profile_msg.header.frame_id = "map"

        # Loop through the array of points and add them
        # to the path message
        for point, velocity in zip(path.T, velocity_profile):
            pose = PoseStamped()
            vel = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            vel.pose.position.x = velocity
            pose.pose.orientation.w = 1.0
            vel.pose.orientation.w = 1.0
            pose.header.stamp = rclpy.time.Time().to_msg()
            vel.header.stamp = pose.header.stamp
            path_msg.poses.append(pose)
            velocity_profile_msg.poses.append(vel)

        self.path_pub.publish(path_msg)
        self.velocity_profile_pub.publish(velocity_profile_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Global Planner Initialized")
    pure_pursuit_node = GlobalPlannerNode()
    rclpy.spin(pure_pursuit_node)
    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
