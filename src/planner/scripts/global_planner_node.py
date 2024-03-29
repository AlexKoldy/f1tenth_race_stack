import rclpy
from rclpy.node import Node
import numpy as np
import os

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
        self.declare_parameter("alpha_min", -0.1)
        self.declare_parameter("alpha_max", 0.1)
        self.declare_parameter("num_waypoints", 1500)
        self.declare_parameter("v_x_min", 2.0)
        self.declare_parameter("v_x_max", 7.0)
        self.declare_parameter("a_x_accel_max", 10.0)
        self.declare_parameter("a_x_decel_max", 4.0)
        self.declare_parameter("a_y_max", 10.0)
        self.declare_parameter("num_iterations", 1)
        self.declare_parameter("trajectory_load_file", "")
        self.declare_parameter("trajectory_save_file", "")
        self.declare_parameter("reoptimize", True)

        # Set up parameters
        self.set_parameters()

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
        trajectory_directory = os.path.join(current_directory, "..", "trajectories")
        trajectory_load_file = os.path.join(
            trajectory_directory, self.trajectory_load_file
        )
        try:
            trajectory_data = np.load(trajectory_load_file)
            self.path = trajectory_directory["path"]
            self.velocity_profile = trajectory_load_file["velocity_profile"]
        except:
            raise Exception(
                f"Either invalid or unfound trajectory file: {trajectory_load_file}"
            )

        # Perform initial trajectory optimization if specified
        if self.reoptimize:
            self.generate_trajectory()

    def set_parameters(self) -> None:
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

        # TODO: grab waypoints another way
        # Make option to not reoptimize, i.e.,
        # save and load waypoints from the optimization
        from funny_splines import FunnySplines

        funny_spline = FunnySplines()
        waypoints = funny_spline.path

        self.path, self.velocity_profile = self.trajectory_builder.generate_trajectory(
            waypoints, self.num_waypoints, self.num_iterations
        )
        # self.trajectory_builder.plot_paths()
        # self.trajectory_builder.plot_optimized_trajectory()

    def timer_callback(self) -> None:
        """"""
        self.set_parameters()

        # If we have a trajectory, publish it
        if self.path:
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
