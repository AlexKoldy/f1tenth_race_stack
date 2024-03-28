import rclpy
from rclpy.node import Node
import numpy as np

# TODO: fix this so that it points to the proper place
# from planner.planner.global_planners.trajectory_builder import TrajectoryBuilder
from trajectory_builder import TrajectoryBuilder

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# TODO: delete this and find way to auto generate this from a map (centerline code)
# from the map
# Yexin's code
def hardcoded_stuff():
    from scipy.interpolate import splprep, splev
    from scipy.interpolate import CubicSpline

    ds = 0.5
    curve_num = 7

    def create_waypoints():
        corner1 = np.array([[6.39, 24.09], [5.55, 24.10], [5.04, 23.27]])
        line1 = np.array([[5.04, 23.27], [-0.36, 2.24]])
        corner2 = np.array([[-0.32, 2.28], [-0.35, 1.14], [0.32, 0.68]])
        line2 = np.array([[0.47, 0.72], [7.32, -1.15]])
        corner3 = np.array([[7.32, -1.15], [8.05, -0.80], [8.24, -0.22]])
        line3 = np.array([[8.24, -0.22], [13.85, 20.79]])
        corner4 = np.array([[13.85, 20.79], [14.04, 21.76], [13.45, 22.30]])
        line4 = np.array([[13.45, 22.30], [6.39, 24.09]])
        corner1_traj = curve_path(corner1, curve_num)
        line1_traj = line_path(line1[0], line1[1], ds)
        corner2_traj = curve_path(corner2, curve_num)
        line2_traj = line_path(line2[0], line2[1], ds)
        corner3_traj = curve_path(corner3, curve_num)
        line3_traj = line_path(line3[0], line3[1], ds)
        corner4_traj = curve_path(corner4, curve_num)
        line4_traj = line_path(line4[0], line4[1], ds)
        waypoints = np.vstack(
            (
                line1_traj,
                corner2_traj,
                line2_traj,
                corner3_traj,
                line3_traj,
                corner4_traj,
                line4_traj,
                corner1_traj,
            )
        )
        return waypoints

    def line_path(start, end, ds):
        # interpolate between the two points
        num = int(np.linalg.norm(end - start) / ds)
        x = np.linspace(start[0], end[0], num)
        y = np.linspace(start[1], end[1], num)
        waypoints = np.vstack((x, y)).T
        return waypoints

    def curve_path(key_points, num):
        # create a spline
        tck, u = splprep(key_points.T, s=2, per=False, k=2)

        # use the spline to create a new set of waypoints
        u_new = np.linspace(u.min(), u.max(), num)
        x_new, y_new = splev(u_new, tck, der=0)

        # combine the new x and y values
        waypoints = np.vstack((x_new, y_new)).T
        return waypoints

    waypoints = create_waypoints()
    x = waypoints[:, 0]
    y = waypoints[:, 1]

    # Generate parameter values (t)
    t = np.linspace(0, 1, len(x))

    # Generate cubic splines for x and y separately
    spline_x = CubicSpline(t, x)
    spline_y = CubicSpline(t, y)

    # Evaluate splines to get interpolated values
    # For example, to get interpolated x and y values at some new t values:
    new_t_values = np.linspace(0, 1, 100)  # Example: 100 new t values
    interpolated_x_values = spline_x(new_t_values)
    interpolated_y_values = spline_y(new_t_values)

    # # Plotting (Optional)
    # import matplotlib.pyplot as plt

    # plt.plot(x, y, "o", label="Waypoints")
    # plt.plot(interpolated_x_values, interpolated_y_values, "-", label="Spline")
    # plt.xlabel("X")
    # plt.ylabel("Y")
    # plt.title("Piecewise Polynomial Spline")
    # plt.legend()
    # plt.grid(True)
    # plt.show()
    waypoints = waypoints[:-1, :].T

    def remove_duplicate_columns(array):
        # Initialize an empty list to store indices of columns to keep
        columns_to_keep = [0]

        # Iterate through each column
        for i in range(1, array.shape[1]):
            # Check if the current column is different from the previous one
            if not np.array_equal(array[:, i], array[:, i - 1]):
                columns_to_keep.append(i)

        # Select only the columns to keep from the original array
        filtered_array = array[:, columns_to_keep]

        return filtered_array

    waypoints = remove_duplicate_columns(waypoints)

    return waypoints


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("global_planner_node")

        # TODO: Make all tune-able parameters ROS params
        # TODO: Create custom PathWithProfile message (for now Imma just set velocity in the z direction)

        # Trajectory builder parameters
        self.alpha_min = -0.25
        self.alpha_max = 0.25
        self.num_waypoints = 100

        # Set up the trajectory builder
        self.trajectory_builder = TrajectoryBuilder(
            self.alpha_min, self.alpha_max, 10, 10, 3, 10
        )

        # Set up topics
        self.path_topic = "/planner/global/path"

        # Set up publishers, subscribers, timers
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # TODO: grab waypoints another way
        # Make option to not reoptimize, i.e.,
        # save and load waypoints from the optimization
        waypoints = hardcoded_stuff()

        self.trajectory_builder.generate_trajectory(waypoints, 100, num_iterations=2)
        self.trajectory_builder.plot_paths()
        self.trajectory_builder.plot_optimized_trajectory()
        # import matplotlib.pyplot as plt

        # fig = plt.figure()
        # ax = fig.add_subplot(111, projection="3d")
        # ax.plot(self.path[0, :], self.path[1, :], velocity_profile)
        # plt.show()

        # self.trajectory_builder.plot_path_with_profile(self.path, velocity_profile)
        exit()

    def timer_callback(self) -> None:
        """"""
        self.publish_trajectory(self.path)

    def publish_trajectory(self, path: np.ndarray) -> None:
        """"""
        # Set up the Path message
        path_msg = Path()
        path_msg.header.stamp = rclpy.time.Time().to_msg()
        path_msg.header.frame_id = "map"

        # Loop through the array of points and add them
        # to the path message
        for point in path.T:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0
            pose.header.stamp = rclpy.time.Time().to_msg()
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Global Planner Initialized")
    pure_pursuit_node = GlobalPlannerNode()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
