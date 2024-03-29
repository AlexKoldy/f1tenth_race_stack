#include <Eigen/Dense>
#include <controller/geometric_controller.hpp>
#include <controller/model_acceleration_pursuit_controller.hpp>
#include <controller/pure_pursuit_controller.hpp>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class GeometricControllerNode : public rclcpp::Node {
 public:
  GeometricControllerNode() : Node("geometric_controller_node") {
    // Declare geometric control parameters
    this->declare_parameter<float>("alpha", 0.0);    // [s]
    this->declare_parameter<float>("beta", 0.0);     // [m]
    this->declare_parameter<float>("ell_min", 0.0);  // [m]
    this->declare_parameter<float>("ell_max", 0.0);  // [m]

    // Declare pure pursuit parameters
    this->declare_parameter<float>("k_p", 0.0);  // [m]

    // Declare geometric controller type
    this->declare_parameter<std::string>("controller_name", "pure_pursuit");

    // Set up controller
    if (this->get_parameter("controller_name").as_string() == "pure_pursuit") {
      controller_ = std::make_unique<PurePursuitController>(
          static_cast<float>(this->get_parameter("alpha").as_double()),
          static_cast<float>(this->get_parameter("beta").as_double()),
          static_cast<float>(this->get_parameter("ell_min").as_double()),
          static_cast<float>(this->get_parameter("ell_max").as_double()),
          static_cast<float>(this->get_parameter("k_p").as_double()));
    } else if (this->get_parameter("controller_name").as_string() ==
               "model_acceleration_pursuit") {
      controller_ = std::make_unique<ModelAccelerationPursuitController>(
          static_cast<float>(this->get_parameter("alpha").as_double()),
          static_cast<float>(this->get_parameter("beta").as_double()),
          static_cast<float>(this->get_parameter("ell_min").as_double()),
          static_cast<float>(this->get_parameter("ell_max").as_double()));
    } else {
      // TODO: handle this
      ;
    }

    // Publishers, subscribers, timers
    drive_pub_ =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            this->drive_topic_, 10);
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        this->path_topic_, 10,
        std::bind(&GeometricControllerNode::path_callback, this,
                  std::placeholders::_1));
    velocity_profile_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        this->velocity_profile_topic_, 10,
        std::bind(&GeometricControllerNode::velocity_profile_callback, this,
                  std::placeholders::_1));
    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->pose_topic_, 10,
        std::bind(&GeometricControllerNode::pose_callback, this,
                  std::placeholders::_1));
    lookahead_point_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            lookahead_marker_topic, 10);
  }

 private:
  // Topics
  std::string path_topic_ = "/planner/global/path";
  std::string velocity_profile_topic_ = "/planner/global/velocity_profile";
  std::string drive_topic_ = "/drive";
  std::string pose_topic_ = "/ego_racecar/odom";
  std::string lookahead_marker_topic = "/controller/viz/lookahead_point";

  // Set up the controller object
  std::unique_ptr<GeometricController> controller_;

  // Publishers, subscribers, timers
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      drive_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr velocity_profile_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      lookahead_point_marker_pub_;

  // Flag to wait for a path before doing calculations
  bool path_received_ = false;
  bool velocity_profile_received_ = false;

  /**
   * @brief Sets the path of the controller.
   * @param path_msg (const nav_msgs::msg::Path::ConstSharedPtr) message
   * containing path information
   */
  void path_callback(const nav_msgs::msg::Path::ConstSharedPtr path_msg) {
    // Find the number of waypoints
    int num_waypoints = path_msg->poses.size();

    // Extract the (x, y) poisitions of each waypoint and convert the path to an
    // Eigen matrix
    Eigen::MatrixXf path(2, num_waypoints);
    for (int i = 0; i < num_waypoints; ++i) {
      path(0, i) = path_msg->poses[i].pose.position.x;
      path(1, i) = path_msg->poses[i].pose.position.y;
    }

    // Set the path
    this->controller_->set_path(path);

    // Set the flag to allow the controller to start working
    this->path_received_ = true;
  }

  void velocity_profile_callback(
      const nav_msgs::msg::Path::ConstSharedPtr velocity_profile_msg) {
    // Find the number of waypoints
    int num_waypoints = velocity_profile_msg->poses.size();

    // Extract the longitudinal velocities from the profile
    Eigen::VectorXf velocity_profile(num_waypoints);
    for (int i = 0; i < num_waypoints; ++i) {
      velocity_profile(i) = velocity_profile_msg->poses[i].pose.position.x;
    }

    // Set the velocity profile
    this->controller_->set_velocity_profile(velocity_profile);

    // Set the flag to allow the controller to start working
    this->velocity_profile_received_ = true;
  }

  /**
   * @brief Handles the main control loop. All ROS2 information is converted to
   * Eigen information for computation.
   * @param pose_msg (const nav_msgs::msg::Odometry::SharedPtr) message
   * containing pose information.
   */
  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg) {
    // Wait for a path and velocity profile to be present before running the
    // controller
    if (this->path_received_ && this->velocity_profile_received_) {
      // Convert the pose to an Eigen Vector2f object for position and
      // an Eigen Matrix3f for orientation
      Eigen::Vector2f p;     // (x, y)-position [m]
      Eigen::Quaternionf q;  // quaternion
      p(0) = pose_msg->pose.pose.position.x;
      p(1) = pose_msg->pose.pose.position.y;
      q.x() = pose_msg->pose.pose.orientation.x;
      q.y() = pose_msg->pose.pose.orientation.y;
      q.z() = pose_msg->pose.pose.orientation.z;
      q.w() = pose_msg->pose.pose.orientation.w;

      // Calculate the steering angle
      std::pair<float, float> control_inputs = this->controller_->step(p, q);
      float speed = control_inputs.first;
      float steering_angle = control_inputs.second;

      // Publish the lookahead point
      this->publish_lookahead_point_marker(
          this->controller_->get_lookahead_point());

      // TODO: clean up
      ackermann_msgs::msg::AckermannDriveStamped drive_msg;
      drive_msg.drive.speed = speed;
      drive_msg.drive.steering_angle = steering_angle;
      this->drive_pub_->publish(drive_msg);

      // std::cout << steering_angle << std::endl;
    }
  }

  /**
   * @brief Publishes the lookahead point marker for visualization.
   * @param lookahead_point (const Eigen::Vector2f) (x,y)-position of the
   * lookahead in the global frame
   */
  void publish_lookahead_point_marker(const Eigen::Vector2f lookahead_point) {
    // std::cout << "Lookahead point: (" << lookahead_point(0) << ", "
    //           << lookahead_point(1) << ")" << std::endl;
    visualization_msgs::msg::Marker lookahead_point_marker_msg;

    lookahead_point_marker_msg.header.frame_id = "map";
    lookahead_point_marker_msg.header.stamp = this->now();
    lookahead_point_marker_msg.ns = "lookahead_point";
    lookahead_point_marker_msg.id = 0;
    lookahead_point_marker_msg.type = visualization_msgs::msg::Marker::SPHERE;
    lookahead_point_marker_msg.action = visualization_msgs::msg::Marker::ADD;

    lookahead_point_marker_msg.pose.position.x = lookahead_point(0);
    lookahead_point_marker_msg.pose.position.y = lookahead_point(1);
    lookahead_point_marker_msg.pose.position.z = 0.0;

    lookahead_point_marker_msg.pose.orientation.x = 0.0;
    lookahead_point_marker_msg.pose.orientation.y = 0.0;
    lookahead_point_marker_msg.pose.orientation.z = 0.0;
    lookahead_point_marker_msg.pose.orientation.w = 1.0;

    lookahead_point_marker_msg.scale.x = 0.1;
    lookahead_point_marker_msg.scale.y = 0.1;
    lookahead_point_marker_msg.scale.z = 0.1;

    lookahead_point_marker_msg.color.r = 0.0;
    lookahead_point_marker_msg.color.g = 1.0;
    lookahead_point_marker_msg.color.b = 0.0;
    lookahead_point_marker_msg.color.a = 1.0;

    lookahead_point_marker_pub_->publish(lookahead_point_marker_msg);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GeometricControllerNode>());
  rclcpp::shutdown();
  return 0;
}