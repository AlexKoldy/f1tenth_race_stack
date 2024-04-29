import rclpy
from rclpy.node import Node
from rclpy import Parameter
import numpy as np
import os

from trajectory_builder import TrajectoryBuilder

import transforms3d as tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray


class LocalLaneChanger(Node):

    def __init__(self):
        super().__init__("local_lane_changer")
        self.window_size = 1
        self.high_val = 2.0
        self.gap_threshold = 2.0
        self.obstacle_width = 0.4

        lidarscan_topic = "/scan"
        drive_topic = "/drive"
        lidar_processed = "/lidar_scan"
        drive_lane = "/drive_lane"

        # Subscribing to LIDAR
        self.subscriber_scan = self.create_subscription(
            LaserScan, lidarscan_topic, self.lidar_callback, 10
        )

        # Publishing to drive
        self.publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        # Publishing the point cloud
        self.laserscan_publisher = self.create_publisher(LaserScan, lidar_processed, 10)

        # Publishing a message - 1: Switch lane; 0: Stay on the same lane
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped, drive_lane, 10
        )

        # Load waypoints
        self.trajectory_load_file = "/home/alko/ese6150/f1tenth_race_stack/src/planner/trajectories/race3/race3_inner.npz"

        self.trajectory_directory = self.trajectory_load_file
        trajectory_load_file = os.path.join(
            self.trajectory_directory, self.trajectory_load_file
        )
        trajectory_data = np.load(trajectory_load_file)
        self.path = trajectory_data["path"]
        self.velocity_profile = trajectory_data["velocity_profile"]
        self.velocity_profile_ = trajectory_data["velocity_profile"]

        # Publishing the path
        self.path_topic = "/planner/global/path"
        self.velocity_profile_topic = "/slowed_velocity_profile"
        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.velocity_profile_pub = self.create_publisher(
            Path, self.velocity_profile_topic, 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Get the current pose
        odom_topic = "/ego_racecar/odom"
        self.pose_subscription = self.create_subscription(
            Odometry, odom_topic, self.pose_callback, 10
        )

        # Publishing points
        point_markers = "/point_markers"
        self.plot_point_publisher = self.create_publisher(
            MarkerArray, point_markers, 10
        )

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

        # Loop through the array of points and add them to the path message
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

        # self.path_pub.publish(path_msg)
        self.velocity_profile_pub.publish(velocity_profile_msg)

    def timer_callback(self) -> None:
        pass
        # self.publish_trajectory(self.path, self.velocity_profile)

    def preprocess_lidar(self, ranges):
        """Preprocess the LiDAR scan array. Expert implementation includes:
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 3m)
        """
        n = len(ranges)
        ranges = np.array(ranges)
        preprocessed_ranges = np.zeros_like(ranges)
        for i in range(n):
            start_index = max(0, i - self.window_size // 2)
            end_index = min(n, i + self.window_size // 2 + 1)
            window_vals = ranges[start_index:end_index]
            window_vals = np.minimum(window_vals, self.high_val)
            # window_vals = np.where(np.logical_or(np.array(window_vals) > 4, np.isnan(window_vals)), 0, window_vals)
            mean_val = np.mean(window_vals)
            preprocessed_ranges[i] = mean_val
        return preprocessed_ranges

    def find_max_obs(self, obs_ranges):
        """Return the start index & end index of the max obs width in obs_ranges"""
        m = len(obs_ranges)
        max_obs = 0
        obs = 0
        max_obs_start = max_obs_end = 0
        self.obs_end = self.obs_start = 0
        for i in range(m):
            obs = 0
            if obs_ranges[i] > self.gap_threshold:
                self.obs_start = i
                obs += 1
                for j in range(i + 1, m):
                    if obs_ranges[j] > self.gap_threshold:
                        obs += 1
                    else:
                        self.obs_end = j
                        break
                if obs > self.obstacle_width:
                    max_obs_start = self.obs_start
                    max_obs_end = self.obs_end

        return max_obs_start, max_obs_start + max_obs - 1

    def find_best_point(self, start_i, end_i, ranges):
        best_index = (start_i + end_i) // 2
        return best_index

    def lidar_callback(self, data):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        ranges = data.ranges
        self.angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
        self.angle_min = data.angle_min
        self.angle_increment = data.angle_increment
        # Considering the scan only between -15 and 15 degrees, esentially reducing the FOV
        angle_90_deg = (np.pi / 180) * 5
        minus_90_deg = (np.pi / 180) * -5
        index_90_deg = int(
            np.floor((angle_90_deg - self.angle_min) / self.angle_increment)
        )
        minus_index_90_deg = int(
            np.ceil((minus_90_deg - self.angle_min) / self.angle_increment)
        )
        clipped_ranges = ranges[minus_index_90_deg:index_90_deg]
        preprocessed_ranges = self.preprocess_lidar(clipped_ranges)

        # Find closest point to LiDAR
        # closest_point_distance = np.min(preprocessed_ranges)
        # closest_point_index = np.argmin(preprocessed_ranges)
        # closest_angle = angles[closest_point_index]

        # Find the obstacle: opponent car
        self.obs_start, self.obs_end = self.find_max_obs(preprocessed_ranges)
        self.obs_start_lidar_range = preprocessed_ranges[self.obs_start]
        self.obs_end_lidar_range = preprocessed_ranges[self.obs_end]
        best_index = self.find_best_point(
            self.obs_start, self.obs_end, preprocessed_ranges
        )
        if preprocessed_ranges[best_index] < self.high_val:
            self.isOppCarOnRaceline = True
        else:
            self.isOppCarOnRaceline = False
        # self.get_logger().info(
        #     'farthest point at "%.2f"' % preprocessed_ranges[best_index]
        # )
        self.dist = preprocessed_ranges[best_index]
        steer_angle = self.angle_min + best_index * self.angle_increment
        self.angle_min = minus_90_deg
        self.angle_max = angle_90_deg

        # Visualizing the lidar scan after processing
        laserscan_msg = LaserScan()
        laserscan_msg.header = data.header
        laserscan_msg.angle_min = self.angle_min
        laserscan_msg.angle_max = self.angle_max
        laserscan_msg.angle_increment = data.angle_increment
        laserscan_msg.time_increment = data.time_increment
        laserscan_msg.scan_time = data.scan_time
        laserscan_msg.range_min = data.range_min
        laserscan_msg.range_max = data.range_max
        laserscan_msg.ranges = preprocessed_ranges[
            self.obs_start : self.obs_end
        ].tolist()  # preprocessed_ranges.tolist()
        laserscan_msg.intensities = data.intensities
        self.laserscan_publisher.publish(laserscan_msg)

    def plot_points(self, points):
        marker_array = MarkerArray()
        for i, point in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "point_marker"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(point[0])
            marker.pose.position.y = float(point[1])
            marker.pose.position.z = 0.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.r = 0.0
            marker.color.a = 0.0
            marker_array.markers.append(marker)
        return marker_array

    def lineLineIntersection(self, A, B, C, D):
        # Plot the 4 points:
        points = np.array((A, B, C, D))
        point_markers = self.plot_points(points)
        self.plot_point_publisher.publish(point_markers)

        # Line AB represented as a1x + b1y = c1
        a1 = B[1] - A[1]
        b1 = A[0] - B[0]
        c1 = a1 * (A[0]) + b1 * (A[1])

        # Line CD represented as a2x + b2y = c2
        a2 = D[1] - C[1]
        b2 = C[0] - D[0]
        c2 = a2 * (C[0]) + b2 * (C[1])

        determinant = a1 * b2 - a2 * b1

        if determinant != 0:
            x = (b2 * c1 - b1 * c2) / determinant
            y = (a1 * c2 - a2 * c1) / determinant
            return np.array([x, y])
        else:
            return None

    def closest_point_on_path(self, current_pose: np.ndarray, path: np.ndarray):
        diff = path - current_pose.reshape((2, 1))
        dist = np.linalg.norm(diff, axis=0)
        closest_idx = np.argmin(dist)

        return closest_idx, path[:, closest_idx]

    def pose_callback(self, pose_msg):
        self.cur_pos = pose_msg.pose.pose.position
        self.cur_quat = pose_msg.pose.pose.orientation
        self.cur_pos = np.array([self.cur_pos.x, self.cur_pos.y])
        self.cur_quat = np.array(
            [self.cur_quat.x, self.cur_quat.y, self.cur_quat.z, self.cur_quat.w]
        )

        # Publish Drive message
        # Check if the intersection point is on the race line:

        # TODO: Get obstacle start and end in robot frame
        self.A = np.array(
            (
                self.obs_start_lidar_range * np.cos(self.angles[self.obs_start]),
                self.obs_start_lidar_range * np.sin(self.angles[self.obs_start]),
            )
        )

        self.B = np.array(
            (
                self.obs_end_lidar_range * np.cos(self.angles[self.obs_end]),
                self.obs_end_lidar_range * np.sin(self.angles[self.obs_end]),
            )
        )

        # Transform the target waypoint to the robot frame
        self.cur_quat = self.cur_quat / np.linalg.norm(self.cur_quat)
        R = tf.quaternions.quat2mat(self.cur_quat)

        # TRANSFORM BOTH GLOBAL POINTS
        obs_start_r = self.A - self.cur_pos
        self.A = R[0:2, 0:2].T @ obs_start_r

        obs_end_r = self.B - self.cur_pos
        self.B = R[0:2, 0:2].T @ obs_end_r

        closest_idx, closest_point = self.closest_point_on_path(self.cur_pos, self.path)
        # Loop through the array of points (till x meters away):

        if self.isOppCarOnRaceline == True:
            msg = AckermannDriveStamped()
            path_rolled = np.roll(self.path, closest_idx)
            for i in range(0, 0 + 20):
                C = path_rolled[:, i]
                D = path_rolled[:, i + 1]
                # print(self.lineLineIntersection(self.A, self.B, C, D).all() != None)
                if self.lineLineIntersection(self.A, self.B, C, D).all() != None:
                    print("Switch lane")
                    msg.drive.speed = 1.0
                    self.drive_publisher.publish(msg)
                    self.state_machine = 1
                    break

        else:
            print("Go on raceline")
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            self.drive_publisher.publish(msg)
            self.state_machine = 0

        if self.state_machine == 1:
            k_p = 1.25
            v_min = np.min(self.velocity_profile)
            v_max = np.max(self.velocity_profile)
            slowed_velocity_profile = np.clip(
                self.velocity_profile - k_p * (1 / self.dist), v_min, v_max
            )
            # print(slowed_velocity_profile[closest_idx])
            # print(self.velocity_profile[closest_idx])
            self.publish_trajectory(self.path, slowed_velocity_profile)
            # self.publish_trajectory(self.path, self.velocity_profile_)
        if self.state_machine == 0:
            # print(self.velocity_profile[closest_idx])
            self.publish_trajectory(self.path, self.velocity_profile)


def main(args=None):
    rclpy.init(args=args)
    print("Lane Change Initialized")
    local_lane_changer = LocalLaneChanger()
    rclpy.spin(local_lane_changer)
    local_lane_changer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
