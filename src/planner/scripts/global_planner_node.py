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
        # 'alpha_min' and 'alpha_max' are the maximum deviations from the
        # original (so they will squeeze or expand it). Rough guide:
        # - 'alpha_min' < 0.0 and 'alpha_max' > 0.0': line can be optimized to expand and contract
        # - 'alpha_min' >= 0.0 and 'alpha_max' > 0.0: line can be squeezed inwards
        # - 'alpha_min' < 0.0 and 'alpha_max' <= 0.0: line can be expanded outwards
        # - 'alpha_min' = 0.0 and 'alpha_max' = 0.0: line stays the same as original,
        #                                            but new velocity profile can be
        #                                            generated for it
        # 'num_waypoints' Determines the number of waypoints the optimized trajectory should
        # have
        # Note: DO NOT change this. It will mess up the Pure Pursuit controller
        # and rebuilding C++ is a pain in the ass, so I don't want to fix the
        # issue right now
        # The following parameters are for generating the velocity profile
        # 'v_x_min' and 'v_x_max' are the maximum longitudinal velocities
        # (the velocities we send to the car via the '/drive' topic).
        # 'v_x_min' will be more likely around sharp corners, 'v_x_max'
        # will be at the beginning of large straightaways.
        # 'a_x_accel_max' is the maximum acceleration the car can go. Setting
        # this to a high value will help ensure the beginning of your straight
        # movements is at a high velocity. 'a_x_decel_max' is the maximum
        # deceleration the car can have (it should be positive). You should keep
        # this value low as we want to ensure that the car slowly ramps down its
        # speed when reaching a curve.
        # 'a_y_max' is the maximum centripital acceleration of the car. It
        # is probably useful to keep this value low and tune it carefully
        # as it seems to be sensitive. The first pass of the velocity profile
        # is generated as v_x = /sqrt{a_y_max * kappa}, where kappa is curvature
        # 'num_iterations' is the amount of times the trajectory generator will run
        # 'trajectory_load_file' will contain the trajectory you want to
        # further optimize.
        # 'trajectory_save_file' will contain the save location of your optimized
        # trajectory. Note: if you are happy with a trajectory, take a photo of
        # all the parameters, and increase the number at the end. This way, you
        # can save "good" trajectories
        # You shouldn't have to worry about 'reoptimize'. Keep it as 'True'
        self.declare_parameter("alpha_min", -0.3)
        self.declare_parameter("alpha_max", -0.3)
        self.declare_parameter("num_waypoints", 1000)
        self.declare_parameter("v_x_min", 2.5)
        self.declare_parameter("v_x_max", 5.0)
        self.declare_parameter("a_x_accel_max", 12.0)
        self.declare_parameter("a_x_decel_max", 3.0)
        self.declare_parameter("a_y_max", 3.0)
        self.declare_parameter("num_iterations", 1)

        self.declare_parameter("trajectory_load_file", "race3_new4.npz")

        self.declare_parameter("trajectory_save_file", "race3_slow.npz")
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
            current_directory, "..", "trajectories", "race3"
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
        self.trajectory_builder.plot_paths()
        self.trajectory_builder.plot_optimized_trajectory()
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
